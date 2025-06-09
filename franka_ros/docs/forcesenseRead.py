import serial
import struct
import time
import threading
import signal
import sys
import glob
from collections import deque
import matplotlib.pyplot as plt

# 初始化全局变量
data_buffers = [deque(maxlen=100) for _ in range(6)]  # 每个通道保存最近 100 个数据点
zero_offsets = [0] * 6  # 每个通道的调零偏移值
data_lock = threading.Lock()  # 数据锁
labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']  # 通道对应的图例名称
running = True  # 控制线程的运行状态
is_zeroing = True  # 是否处于调零阶段

# 添加信号处理器
def signal_handler(sig, frame):
    global running
    print('\n信号中断检测到，正在安全停止...')
    running = False
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def find_usb_port():
    """
    自动检测可用的USB串口
    """
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            if ser.is_open:
                print(f"找到USB设备: {port}")
                ser.close()
                return port
        except (OSError, serial.SerialException):
            continue
    
    print("未找到可用的USB串口设备")
    return None

def setup_serial(port='/dev/ttyUSB0', baudrate=115200):
    """
    初始化串口连接
    """
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    if ser.is_open:
        print(f"串口 {port} 已打开")
    else:
        print(f"无法打开串口 {port}")
    return ser


def send_command(ser, command):
    """
    发送命令到传感器
    """
    if not command.endswith('\r\n'):
        command += '\r\n'
    ser.write(command.encode())
    print(f"发送: {command.strip()}")


def reverse_bytes(data):
    """
    按字节反转顺序
    """
    return bytes(data[::-1])


def parse_sensor_data(data):
    """
    解析传感器返回的六通道力数据
    """
    try:
        # 检查帧头
        if data[:2] != b'\xAA\x55':
            print("数据头无效")
            print(f"原始数据: {data.hex()}")  # 打印原始数据用于调试
            return None, None

        # 获取长度和包编号
        package_no = struct.unpack('>H', data[4:6])[0]

        # 按需反转每个通道的数据字节
        forces = []
        for i in range(6):
            raw_bytes = data[6 + i * 4:10 + i * 4]  # 每个通道 4 字节
            reversed_bytes = reverse_bytes(raw_bytes)
            force = struct.unpack('>f', reversed_bytes)[0]
            forces.append(force)

        return package_no, forces
    except Exception as e:
        print(f"解析错误: {e}")
        print(f"原始数据: {data.hex()}")  # 打印原始数据用于调试
        return None, None


def sync_data(ser):
    """
    数据同步：找到帧头 AA 55
    """
    while True:
        byte = ser.read(1)  # 逐字节读取
        if byte == b'\xAA':
            next_byte = ser.read(1)
            if next_byte == b'\x55':
                return b'\xAA\x55'  # 找到帧头


def is_valid_force(force, min_val=-100, max_val=100):
    """
    校验力值是否在合理范围内
    """
    return min_val <= force <= max_val


def read_data(ser):
    """
    数据采集线程
    """
    global running, is_zeroing, zero_offsets
    zero_data = [[] for _ in range(6)]  # 用于收集调零数据

    try:
        while running:
            # 数据同步
            sync_data(ser)
            raw_data = ser.read(29)  # 帧头后面 29 字节
            if len(raw_data) < 29:
                print("数据包不完整")
                continue

            raw_data = b'\xAA\x55' + raw_data  # 补充帧头
            package_no, forces = parse_sensor_data(raw_data)
            if forces:
                if is_zeroing:
                    # 调零阶段：收集数据
                    for i, force in enumerate(forces):
                        zero_data[i].append(force)
                    if len(zero_data[0]) >= 100:  # 收集足够数据后计算零偏值
                        zero_offsets = [sum(zero_data[i]) / len(zero_data[i]) for i in range(6)]
                        print(f"调零完成，零偏值: {zero_offsets}")
                        is_zeroing = False
                else:
                    # 去零后的数据
                    adjusted_forces = [forces[i] - zero_offsets[i] for i in range(6)]

                    # 校验力值并更新缓冲区
                    filtered_forces = [force if is_valid_force(force) else 0 for force in adjusted_forces]
                    with data_lock:
                        for i, force in enumerate(filtered_forces):
                            data_buffers[i].append(force)

                    print(f"Package No: {package_no}, Adjusted Forces: {filtered_forces}")
    except KeyboardInterrupt:
        print("数据采集线程终止")
    finally:
        running = False


def plot_data():
    """
    实时绘图函数
    """
    plt.ion()
    fig, ax = plt.subplots()
    lines = []

    for i in range(6):
        line, = ax.plot([], [], label=labels[i])
        lines.append(line)

    ax.set_xlim(0, 100)
    ax.set_ylim(-10, 10)  # 根据数据范围调整 Y 轴
    ax.set_title("Real-Time 6D Force Visualization")
    ax.set_xlabel("Time (Frames)")
    ax.set_ylabel("Force/Moment")
    ax.legend()

    try:
        while running:
            with data_lock:
                for i, line in enumerate(lines):
                    y_data = list(data_buffers[i])
                    x_data = range(len(y_data))
                    line.set_xdata(x_data)
                    line.set_ydata(y_data)

            fig.canvas.flush_events()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("绘图线程终止")
    finally:
        plt.ioff()
        plt.show(block=True)  # 确保窗口显示


def main():
    global running
    
    # 自动检测USB端口
    port = find_usb_port()
    if port is None:
        print("错误：无法找到力传感器USB设备")
        return
    
    # 设置串口参数
    ser = setup_serial(port=port, baudrate=115200)
    if not ser.is_open:
        print("错误：无法打开串口")
        return

    try:
        # 设置采样率
        send_command(ser, 'AT+SMPF=300')
        time.sleep(0.5)
        send_command(ser, 'AT+GSD')

        # 创建数据采集线程
        read_thread = threading.Thread(target=read_data, args=(ser,), daemon=True)

        # 启动数据采集线程
        read_thread.start()

        # 在主线程中运行绘图
        plot_data()
        
    except KeyboardInterrupt:
        print("\n检测到中断信号")
    except Exception as e:
        print(f"运行时错误: {e}")
    finally:
        print("正在清理资源...")
        running = False
        try:
            send_command(ser, 'AT+STOP')  # 停止连续传输
            time.sleep(0.1)
        except:
            pass
        
        if ser.is_open:
            ser.close()
            print("串口已关闭")
        
        print("程序已安全退出")


if __name__ == '__main__':
    main()
