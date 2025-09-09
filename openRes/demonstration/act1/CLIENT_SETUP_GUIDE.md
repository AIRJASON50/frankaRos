# ACT分布式推理系统 - 客户端设置指南

## 📋 项目背景与进度

### 系统架构
```
┌─────────────────┐         ┌─────────────────┐
│   客户端电脑     │  网络   │   服务器电脑     │
│  (配置较低)      │ ◀────▶ │   (高性能GPU)   │
│                 │         │                 │
│ 🎮 运行仿真环境  │         │ 🧠 AI模型推理   │
│ 📷 收集图像数据  │         │ ⚡ GPU加速计算   │
│ 🎯 执行动作      │         │ 📤 返回决策     │
└─────────────────┘         └─────────────────┘
```

### 当前状态
- ✅ **服务器端已完成**: ACT推理服务器运行在 `ws://10.16.49.124:8765`
- ✅ **模型已加载**: policy_epoch_700_seed_0.ckpt (训练700轮)
- ✅ **网络已配置**: 局域网通信，延迟<10ms
- 🔄 **客户端待配置**: 需要在此电脑上设置仿真客户端

## 🛠️ 客户端环境配置

### 前提条件
- 客户端电脑已有ACT环境 (曾运行过数据集生成与仿真)
- Python 3.9+ 环境
- 可访问服务器IP: `10.16.49.124`

### 步骤1: 检查现有环境
```bash
# 检查Python环境
python --version
# 应该显示 Python 3.9+

# 检查ACT相关包
python -c "import numpy, cv2; print('基础包OK')"
python -c "import torch; print('PyTorch OK')"

# 检查网络连通性
ping 10.16.49.124
```

### 步骤2: 安装分布式推理依赖
```bash
# 安装WebSocket客户端包
pip install websockets==11.0.3
pip install opencv-python==4.8.1.78

# 验证安装
python -c "import websockets; print('WebSocket已安装')"
```

## 📁 客户端代码设置

### 创建项目目录
```bash
# 在客户端电脑上创建目录
mkdir -p ~/act_distributed_client
cd ~/act_distributed_client
```

### 下载客户端代码
将以下代码保存为 `simulation_client.py`:

```python
#!/usr/bin/env python3
"""
ACT分布式推理客户端
在低配置电脑上运行仿真，通过网络连接服务器进行ACT推理
"""

import asyncio
import websockets
import json
import base64
import numpy as np
import cv2
import time
import logging
import argparse
from collections import deque
from datetime import datetime

# 导入仿真环境 - 根据你的ACT环境调整路径
import sys
# sys.path.append('path/to/your/act')  # 修改为你的ACT路径
from sim_env import make_sim_env
from constants import SIM_TASK_CONFIGS

class SimulationClient:
    def __init__(self, server_url, task_name='sim_transfer_cube_scripted', 
                 max_timesteps=400, render=True):
        self.server_url = server_url
        self.task_name = task_name
        self.max_timesteps = max_timesteps
        self.render = render
        
        # 动作缓冲区
        self.action_buffer = deque(maxlen=200)
        self.action_request_threshold = 20  # 剩余动作少于20时请求新的
        
        # 序列ID管理
        self.sequence_id = 0
        self.pending_requests = {}
        
        # 统计信息
        self.episode_count = 0
        self.total_steps = 0
        self.network_latencies = []
        self.inference_times = []
        
        # 设置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
        # 创建仿真环境
        self.env = None
        self.init_environment()
    
    def init_environment(self):
        """初始化仿真环境"""
        try:
            self.logger.info(f"Initializing simulation environment: {self.task_name}")
            
            # 获取任务配置
            task_config = SIM_TASK_CONFIGS[self.task_name]
            
            # 创建环境
            self.env = make_sim_env(self.task_name)
            self.camera_names = task_config['camera_names']
            
            self.logger.info("Simulation environment initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize environment: {e}")
            raise
    
    def encode_image(self, image, quality=80):
        """编码图像为base64"""
        try:
            # 调整图像大小以减少传输量
            if image.shape[:2] != (240, 320):
                image = cv2.resize(image, (320, 240))
            
            # 转换BGR到RGB (如果需要)
            if len(image.shape) == 3 and image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # JPEG编码
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, buffer = cv2.imencode('.jpg', image, encode_param)
            
            # Base64编码
            encoded = base64.b64encode(buffer).decode('utf-8')
            return encoded
            
        except Exception as e:
            self.logger.error(f"Failed to encode image: {e}")
            return None
    
    def prepare_request(self, obs):
        """准备推理请求数据"""
        try:
            # 提取机器人状态
            qpos = obs['qpos']  # 关节位置
            qvel = obs['qvel']  # 关节速度
            
            # 提取图像 (假设只有一个相机)
            camera_image = None
            if 'images' in obs and self.camera_names:
                camera_name = self.camera_names[0]
                if camera_name in obs['images']:
                    image = obs['images'][camera_name]
                    camera_image = self.encode_image(image)
            
            # 构造请求
            self.sequence_id += 1
            request = {
                'timestamp': time.time(),
                'sequence_id': self.sequence_id,
                'robot_state': {
                    'joint_positions': qpos.tolist(),
                    'joint_velocities': qvel.tolist()
                },
                'camera_image': camera_image
            }
            
            return request
            
        except Exception as e:
            self.logger.error(f"Failed to prepare request: {e}")
            return None
    
    async def request_actions(self, websocket, obs):
        """请求新的动作序列"""
        try:
            request = self.prepare_request(obs)
            if request is None:
                return False
            
            # 记录请求时间
            request_time = time.time()
            self.pending_requests[request['sequence_id']] = request_time
            
            # 发送请求
            await websocket.send(json.dumps(request))
            self.logger.debug(f"Sent action request #{request['sequence_id']}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to request actions: {e}")
            return False
    
    def process_response(self, response_data):
        """处理服务器响应"""
        try:
            sequence_id = response_data['sequence_id']
            actions = np.array(response_data['actions'])
            
            # 计算网络延迟
            if sequence_id in self.pending_requests:
                request_time = self.pending_requests[sequence_id]
                latency = time.time() - request_time
                self.network_latencies.append(latency)
                del self.pending_requests[sequence_id]
                
                self.logger.info(f"Received actions #{sequence_id}: "
                               f"{len(actions)} steps, latency: {latency:.3f}s")
            
            # 添加动作到缓冲区
            for action in actions:
                self.action_buffer.append(action)
            
            # 记录推理时间
            if 'inference_time' in response_data:
                self.inference_times.append(response_data['inference_time'])
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to process response: {e}")
            return False
    
    async def run_episode(self, websocket):
        """运行一个episode"""
        self.logger.info(f"Starting episode #{self.episode_count + 1}")
        
        try:
            # 重置环境
            obs = self.env.reset()
            episode_reward = 0
            step_count = 0
            
            # 初始动作请求
            await self.request_actions(websocket, obs)
            
            # 等待初始动作
            while len(self.action_buffer) == 0:
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=10.0)
                    response_data = json.loads(response)
                    
                    if 'error' in response_data:
                        self.logger.error(f"Server error: {response_data['error']}")
                        return False
                    
                    self.process_response(response_data)
                    
                except asyncio.TimeoutError:
                    self.logger.error("Timeout waiting for initial actions")
                    return False
            
            # 运行episode
            while step_count < self.max_timesteps:
                # 检查是否需要请求新动作
                if len(self.action_buffer) < self.action_request_threshold:
                    await self.request_actions(websocket, obs)
                
                # 处理服务器响应 (非阻塞)
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=0.001)
                    response_data = json.loads(response)
                    
                    if 'error' in response_data:
                        self.logger.warning(f"Server error: {response_data['error']}")
                    else:
                        self.process_response(response_data)
                        
                except asyncio.TimeoutError:
                    pass  # 没有新消息，继续执行
                
                # 执行动作
                if len(self.action_buffer) > 0:
                    action = self.action_buffer.popleft()
                    obs, reward, done, info = self.env.step(action)
                    
                    episode_reward += reward
                    step_count += 1
                    self.total_steps += 1
                    
                    # 渲染
                    if self.render:
                        self.env.render()
                    
                    # 检查episode结束
                    if done:
                        self.logger.info(f"Episode finished: reward={episode_reward:.3f}, steps={step_count}")
                        break
                        
                else:
                    # 没有可用动作，等待
                    self.logger.warning("Action buffer empty, waiting...")
                    await asyncio.sleep(0.01)
            
            self.episode_count += 1
            return True
            
        except Exception as e:
            self.logger.error(f"Episode error: {e}")
            return False
    
    async def run_simulation(self):
        """运行仿真主循环"""
        self.logger.info(f"Connecting to server: {self.server_url}")
        
        try:
            async with websockets.connect(self.server_url) as websocket:
                self.logger.info("Connected to inference server")
                
                # 运行多个episodes
                success_count = 0
                max_episodes = 5  # 可配置
                
                for episode in range(max_episodes):
                    success = await self.run_episode(websocket)
                    if success:
                        success_count += 1
                    
                    # 清理缓冲区
                    self.action_buffer.clear()
                    self.pending_requests.clear()
                    
                    # 间隔
                    if episode < max_episodes - 1:
                        await asyncio.sleep(1.0)
                
                # 打印统计信息
                self.print_statistics(success_count, max_episodes)
                
        except Exception as e:
            self.logger.error(f"Connection error: {e}")
            return False
        
        return True
    
    def print_statistics(self, success_count, total_episodes):
        """打印统计信息"""
        print("\n" + "="*50)
        print("📊 仿真统计信息")
        print("="*50)
        print(f"成功Episodes: {success_count}/{total_episodes}")
        print(f"总步数: {self.total_steps}")
        
        if self.network_latencies:
            avg_latency = np.mean(self.network_latencies)
            max_latency = np.max(self.network_latencies)
            print(f"网络延迟: 平均 {avg_latency:.3f}s, 最大 {max_latency:.3f}s")
        
        if self.inference_times:
            avg_inference = np.mean(self.inference_times)
            print(f"推理时间: 平均 {avg_inference:.3f}s")
        
        print("="*50 + "\n")

def main():
    parser = argparse.ArgumentParser(description='ACT分布式推理客户端')
    parser.add_argument('--server_url', type=str, required=True,
                       help='服务器WebSocket URL (如: ws://10.16.49.124:8765)')
    parser.add_argument('--task_name', type=str, 
                       default='sim_transfer_cube_scripted',
                       help='仿真任务名称')
    parser.add_argument('--max_timesteps', type=int, default=400,
                       help='每个episode最大步数')
    parser.add_argument('--no_render', action='store_true',
                       help='禁用渲染 (加快仿真速度)')
    
    args = parser.parse_args()
    
    # 创建客户端
    client = SimulationClient(
        server_url=args.server_url,
        task_name=args.task_name,
        max_timesteps=args.max_timesteps,
        render=not args.no_render
    )
    
    # 运行仿真
    try:
        asyncio.run(client.run_simulation())
    except KeyboardInterrupt:
        print("\n🛑 仿真已停止")
    except Exception as e:
        print(f"❌ 客户端错误: {e}")

if __name__ == "__main__":
    main()
```

## 🔧 配置文件设置

### 创建启动脚本
将以下内容保存为 `start_client.sh`:

```bash
#!/bin/bash
# ACT分布式推理客户端启动脚本

# 服务器配置
SERVER_IP="10.16.49.124"
SERVER_PORT="8765"
SERVER_URL="ws://${SERVER_IP}:${SERVER_PORT}"

echo "🚀 启动ACT分布式推理客户端"
echo "📡 连接服务器: ${SERVER_URL}"
echo "=================================="

# 检查网络连通性
echo "🔍 检查网络连通性..."
if ping -c 1 ${SERVER_IP} > /dev/null; then
    echo "✅ 网络连接正常"
else
    echo "❌ 无法连接到服务器 ${SERVER_IP}"
    exit 1
fi

# 检查依赖
echo "🔍 检查环境依赖..."
python -c "import websockets, numpy, cv2" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ 依赖包检查通过"
else
    echo "❌ 缺少必要依赖包，请先安装websockets"
    exit 1
fi

# 启动客户端
echo "🎮 启动仿真客户端..."
python simulation_client.py --server_url ${SERVER_URL} --task_name sim_transfer_cube_scripted

echo "🛑 客户端已退出"
```

给脚本执行权限:
```bash
chmod +x start_client.sh
```

## 🚀 运行步骤

### 步骤1: 确保服务器运行
在服务器电脑上确认推理服务器正在运行:
```
✅ 服务器状态: ws://10.16.49.124:8765
⚡ 状态: 等待客户端连接...
```

### 步骤2: 修改ACT路径
编辑 `simulation_client.py` 第21行，修改为你的ACT路径:
```python
sys.path.append('path/to/your/act')  # 修改为实际路径
```

### 步骤3: 启动客户端
```bash
# 方法1: 使用启动脚本
./start_client.sh

# 方法2: 直接运行
python simulation_client.py --server_url ws://10.16.49.124:8765
```

### 步骤4: 观察运行状态
正常运行时应该看到:
```
📊 成功连接到推理服务器
🎮 开始episode #1
📡 网络延迟: 平均 0.005s
🧠 推理时间: 平均 0.030s
```

## 🔧 故障排除

### 问题1: 连接失败
```bash
# 检查网络
ping 10.16.49.124

# 检查端口
telnet 10.16.49.124 8765
```

### 问题2: ACT模块导入失败
```bash
# 检查路径设置
python -c "import sys; print(sys.path)"
# 确认ACT模块路径在sys.path中
```

### 问题3: 性能问题
- 网络延迟>50ms: 检查网络连接
- 推理延迟>100ms: 服务器GPU可能忙碌
- 动作缓冲区空: 降低action_request_threshold

## 📋 检查清单

- [ ] 客户端Python环境正常
- [ ] websockets包已安装
- [ ] 可以ping通服务器IP
- [ ] ACT模块路径正确设置
- [ ] simulation_client.py代码已保存
- [ ] start_client.sh脚本已创建并给权限
- [ ] 服务器端正在运行
- [ ] 客户端成功连接并运行仿真

## 📞 支持信息

**服务器信息:**
- IP地址: 10.16.49.124
- 端口: 8765
- 协议: WebSocket
- 模型: ACT epoch 700

**预期性能:**
- 网络延迟: 2-10ms
- 推理延迟: 20-50ms
- 总延迟: 25-60ms
- Chunk大小: 100步动作

**成功指标:**
- 客户端连接成功
- 收到动作序列
- 仿真正常运行
- 网络延迟稳定
