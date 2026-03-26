"""Offline Path Downloader for Crane Simulation

交互式轨迹数据下发工具
- 列出 data 目录中的 CSV 文件
- 选择轨迹文件后下发至 PLC
- 完成后可选择退出或加载新轨迹

Usage:
    python offline_path.py
"""

import snap7
from snap7.util import set_int, set_real
import pandas as pd
import struct
import time
import glob
from pathlib import Path


class PLCTrajectoryTool:
    """PLC 轨迹下发工具"""
    
    def __init__(self, ip, rack=0, slot=1, db_number=10):
        self.client = snap7.client.Client()
        self.ip = ip
        self.rack = rack
        self.slot = slot
        self.db_num = db_number
        self.batch_id = 1
        self.connected = False
        
    def connect(self):
        """连接到 PLC"""
        try:
            self.client.connect(self.ip, self.rack, self.slot)
            self.connected = True
            print(f"✓ 成功连接至 PLC: {self.ip}")
            return True
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            self.connected = False
            return False

    def send_complete_trajectory(self, points_df):
        """
        将完整轨迹发送至 PLC (自动分批处理)
        points_df 包含列: x, y, z, vx, vy, vz
        """
        total_points = len(points_df)
        points_sent = 0
        
        print(f"\n开始下发轨迹数据...")
        print(f"总点数: {total_points}")
        print("-" * 70)
        
        # 按 100 点分批
        for batch_start in range(0, total_points, 100):
            batch_end = min(batch_start + 100, total_points)
            batch_df = points_df.iloc[batch_start:batch_end]
            
            # 创建缓冲区并发送
            buffer = bytearray(2500)
            
            # 写入 BatchID 和 MovePointNum
            
            set_int(buffer, 10, len(batch_df))
            
            # 写入点数据
            current_offset = 14
            for _, row in batch_df.iterrows():
                set_real(buffer, current_offset,      float(row['x']))
                set_real(buffer, current_offset + 4,  float(row['y']))
                set_real(buffer, current_offset + 8,  float(row['z']))
                set_real(buffer, current_offset + 12, float(row['vx']))
                set_real(buffer, current_offset + 16, float(row['vy']))
                set_real(buffer, current_offset + 20, float(row['vz']))
                current_offset += 24
            
            # 发送至 PLC
            set_int(buffer, 2414, self.batch_id)
            write_size = current_offset - 8
            self.client.db_write(self.db_num, 8, buffer[8:2500])
            
            points_sent += len(batch_df)
            progress = (points_sent / total_points) * 100
            
            print(f"批次 {self.batch_id}: 已发送 {len(batch_df)} 点 | "
                  f"总进度 {points_sent}/{total_points} ({progress:.1f}%)")
            
            self.batch_id += 1
            time.sleep(0.1)  # 批次间延时
        
        print("-" * 70)
        print(f"✓ 轨迹下发完成！服费发送 {points_sent} 个点")
        return points_sent

    def disconnect(self):
        """断开 PLC 连接"""
        if self.connected:
            self.client.disconnect()
            self.connected = False
            print("PLC 已断开连接")


def list_available_csv_files():
    """列出 data 目录中的 CSV 文件"""
    files = sorted(glob.glob('data/*.csv'))
    
    if not files:
        print("未找到任何 CSV 文件 (data/*.csv)")
        return None
    
    print("\n可用的轨迹文件:")
    for i, f in enumerate(files):
        size = Path(f).stat().st_size / 1024  # KB
        # 计算点数（假设 CSV 有表头）
        try:
            df = pd.read_csv(f)
            points = len(df)
            print(f"  [{i}] {Path(f).name:40s} ({points:6d} 点) ({size:8.1f} KB)")
        except:
            print(f"  [{i}] {Path(f).name:40s} (读取失败) ({size:8.1f} KB)")
    
    return files


def validate_and_load_csv(filename_or_index, available_files=None):
    """验证并加载 CSV 文件
    
    支持三种输入形式：
    1. 数字索引：选择列表中的文件 (e.g., "0")
    2. 完整路径：直接加载指定文件 (e.g., "data/trajectory_2001.csv")
    3. 文件名：查找匹配的文件 (e.g., "trajectory_2001.csv")
    """
    input_str = filename_or_index.strip()
    
    # 检查是否为数字索引
    if input_str.isdigit():
        if available_files is None:
            print("❌ 没有可用文件列表")
            return None
        
        idx = int(input_str)
        if 0 <= idx < len(available_files):
            filepath = available_files[idx]
        else:
            print(f"❌ 索引超出范围 [0-{len(available_files)-1}]")
            return None
    else:
        # 检查完整路径是否存在
        if Path(input_str).exists() and input_str.endswith('.csv'):
            filepath = input_str
        else:
            # 尝试在 data 目录中查找
            data_path = Path('data') / input_str
            if data_path.exists():
                filepath = str(data_path)
            else:
                # 尝试模糊匹配
                matches = glob.glob(f"data/*{input_str}*csv")
                if matches:
                    if len(matches) == 1:
                        filepath = matches[0]
                    else:
                        print(f"❌ 找到多个匹配文件:")
                        for m in matches:
                            print(f"    {m}")
                        return None
                else:
                    print(f"❌ 找不到文件: {input_str}")
                    return None
    
    # 加载 CSV 文件
    try:
        df = pd.read_csv(filepath)
        print(f"✓ 已加载: {filepath} ({len(df)} 点)")
        return df
    except Exception as e:
        print(f"❌ 读取 CSV 失败: {e}")
        return None


def interactive_mode(tool):
    """交互模式 - 选择轨迹并下发"""
    
    print("\n" + "="*70)
    print("轨迹下发交互模式")
    print("="*70)
    print("命令说明:")
    print("  - 输入数字:   从列表中选择文件 (例: 0, 1, 2)")
    print("  - 输入文件名: 分析指定文件")
    print("  - 输入 'ls':  列出所有可用文件")
    print("  - 输入 'quit' 或 'exit': 退出程序")
    print("="*70 + "\n")
    
    available_files = list_available_csv_files()
    
    while True:
        try:
            user_input = input("\n请输入文件编号或文件名 > ").strip()
            
            # 检查退出命令
            if user_input.lower() in ['quit', 'exit', 'q']:
                print("✓ 程序已退出")
                break
            
            # 检查列表命令
            if user_input.lower() == 'ls':
                available_files = list_available_csv_files()
                continue
            
            # 验证并加载文件
            if not user_input:
                continue
            
            df = validate_and_load_csv(user_input, available_files)
            if df is not None:
                # 验证 CSV 格式
                required_cols = ['x', 'y', 'z', 'vx', 'vy', 'vz']
                if all(col in df.columns for col in required_cols):
                    # 下发轨迹
                    tool.send_complete_trajectory(df)
                    # 重新刷新文件列表
                    available_files = list_available_csv_files()
                else:
                    print(f"❌ CSV 格式错误，需要列: {required_cols}")
                    print(f"   实际列: {list(df.columns)}")
        
        except KeyboardInterrupt:
            print("\n\n✓ 程序已退出")
            break
        except Exception as e:
            print(f"❌ 错误: {e}")


def main():
    """主函数"""
    # PLC 配置
    PLC_IP = '192.168.111.240'
    
    print("="*70)
    print("离线轨迹下发工具")
    print("="*70)
    
    # 创建工具实例
    tool = PLCTrajectoryTool(PLC_IP)
    
    # 连接 PLC
    if not tool.connect():
        print("无法连接至 PLC，退出")
        return
    
    # 进入交互模式
    try:
        interactive_mode(tool)
    finally:
        tool.disconnect()


if __name__ == "__main__":
    main()