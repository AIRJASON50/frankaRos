#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Force_z数据绘图工具
从CSV日志文件中提取force_z字段并生成图像
用法: python3 plot_force_z_from_csv.py <csv_file_path>
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

def plot_force_z_from_csv(csv_file_path, save_image=True):
    """
    从CSV文件中读取并绘制force_z数据
    
    Args:
        csv_file_path (str): CSV文件路径
        save_image (bool): 是否保存图像
    """
    try:
        print(f"Reading CSV file: {csv_file_path}")
        
        # 读取CSV文件，跳过注释行
        # CSV格式: time,phase,pos_x,pos_y,pos_z,force_x,force_y,force_z,raw_force_z,force_magnitude,energy_level,energy_scale_factor
        df = pd.read_csv(csv_file_path, comment='#')
        
        print(f"Successfully read {len(df)} rows of data")
        print(f"CSV columns: {list(df.columns)}")
        
        # 确认必要的列存在
        required_columns = ['time', 'force_z', 'raw_force_z']
        missing_columns = [col for col in required_columns if col not in df.columns]
        if missing_columns:
            print(f"Error: Missing required columns: {missing_columns}")
            return
        
        # 转换时间为相对时间（从0开始）
        if len(df) > 0:
            start_time = df['time'].iloc[0]
            df['relative_time'] = df['time'] - start_time
        else:
            print("Error: No data found in CSV file")
            return
        
        # 数据统计 - 修复pandas兼容性问题
        force_z_stats = {
            'count': len(df),
            'mean': float(df['force_z'].mean()),
            'std': float(df['force_z'].std()),
            'min': float(df['force_z'].min()),
            'max': float(df['force_z'].max())
        }
        
        raw_force_z_stats = {
            'count': len(df),
            'mean': float(df['raw_force_z'].mean()),
            'std': float(df['raw_force_z'].std()),
            'min': float(df['raw_force_z'].min()),
            'max': float(df['raw_force_z'].max())
        }
        
        print(f"\n=== Force_z Data Statistics ===")
        print(f"Data points: {force_z_stats['count']}")
        print(f"Force_z (Filtered) - Mean: {force_z_stats['mean']:.3f} N, Std: {force_z_stats['std']:.3f} N")
        print(f"Force_z (Filtered) - Range: [{force_z_stats['min']:.3f}, {force_z_stats['max']:.3f}] N")
        print(f"Raw_Force_z - Mean: {raw_force_z_stats['mean']:.3f} N, Std: {raw_force_z_stats['std']:.3f} N")
        print(f"Raw_Force_z - Range: [{raw_force_z_stats['min']:.3f}, {raw_force_z_stats['max']:.3f}] N")
        
        # 创建图像
        plt.figure(figsize=(12, 8))
        
        # 绘制滤波后的force_z
        plt.subplot(2, 1, 1)
        plt.plot(df['relative_time'].values, df['force_z'].values, 'b-', linewidth=1.5, alpha=0.8, label='Force Z (Filtered)')
        plt.plot(df['relative_time'].values, df['raw_force_z'].values, 'r-', linewidth=1, alpha=0.6, label='Force Z (Raw)')
        
        # ✅ English labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Force Z (N)')
        plt.title('Force Z Data Comparison - Filtered vs Raw')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # 添加统计信息文本框
        stats_text = f'Filtered: μ={force_z_stats["mean"]:.3f}N, σ={force_z_stats["std"]:.3f}N\n'
        stats_text += f'Raw: μ={raw_force_z_stats["mean"]:.3f}N, σ={raw_force_z_stats["std"]:.3f}N'
        plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # 绘制滤波效果（差值）
        plt.subplot(2, 1, 2)
        noise_diff = df['raw_force_z'].values - df['force_z'].values  # ✅ 转换为numpy数组避免索引问题
        plt.plot(df['relative_time'].values, noise_diff, 'g-', linewidth=1, alpha=0.7, label='Filtering Effect (Raw - Filtered)')
        
        # ✅ English labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Force Difference (N)')
        plt.title('Filtering Effect - Noise Reduction')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # 添加噪声统计 - 修复兼容性
        noise_stats_text = f'Noise μ={float(np.mean(noise_diff)):.3f}N, σ={float(np.std(noise_diff)):.3f}N\n'
        noise_stats_text += f'Noise range: [{float(np.min(noise_diff)):.3f}, {float(np.max(noise_diff)):.3f}]N'
        plt.text(0.02, 0.98, noise_stats_text, transform=plt.gca().transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
        
        plt.tight_layout()
        
        # 保存图像
        if save_image:
            # 生成保存文件名
            base_name = os.path.splitext(os.path.basename(csv_file_path))[0]
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = f"force_z_plot_{base_name}_{timestamp}.png"
            
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"\nImage saved: {save_path}")
        
        # 显示图像
        plt.show()
        
        return True
        
    except Exception as e:
        print(f"Error processing CSV file: {e}")
        return False

def main():
    """主函数"""
    if len(sys.argv) != 2:
        print("用法: python3 plot_force_z_from_csv.py <csv_file_path>")
        print("示例: python3 plot_force_z_from_csv.py /path/to/force_data_20250708_142324.csv")
        sys.exit(1)
    
    csv_file_path = sys.argv[1]
    
    # 检查文件是否存在
    if not os.path.exists(csv_file_path):
        print(f"错误: 文件不存在: {csv_file_path}")
        sys.exit(1)
    
    # 处理文件
    success = plot_force_z_from_csv(csv_file_path, save_image=True)
    
    if success:
        print("Force_z数据绘图完成!")
    else:
        print("处理失败!")
        sys.exit(1)

if __name__ == '__main__':
    main() 