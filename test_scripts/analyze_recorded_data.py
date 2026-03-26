"""Offline analysis tool for recorded waveform data.

Usage:
    python analyze_recorded_data.py                    # 分析最新数据，进入交互模式
    python analyze_recorded_data.py data/recorded_data_*.json  # 分析指定文件，进入交互模式

交互命令：
    0, 1, 2...     - 从列表中选择文件
    recorded_data_* - 输入文件名分析
    ls             - 列出所有可用文件
    quit/exit      - 退出程序
"""

import json
import matplotlib.pyplot as plt
import glob
import sys
from pathlib import Path
import numpy as np


def load_and_plot_data(filename):
    """加载 JSON 数据并绘制图表"""
    print(f"\n加载数据文件: {filename}")
    
    with open(filename, 'r') as f:
        data = json.load(f)
    
    t = np.array(data['t'])
    num_points = len(t)
    
    print(f"数据点数: {num_points}")
    print(f"时间范围: {t[0]:.3f}s ~ {t[-1]:.3f}s (总时长: {t[-1] - t[0]:.3f}s)")
    
    # ─────────────────────────
    # 图1：XYZ 运动控制
    # ─────────────────────────
    fig1, axes1 = plt.subplots(3, 1, figsize=(14, 10), constrained_layout=True)
    fig1.suptitle(f'Offline Analysis: Motion Control - {Path(filename).stem}', 
                  fontsize=14, fontweight='bold')
    
    for idx, axis_name in enumerate(['x', 'y', 'z']):
        ax = axes1[idx]
        ax.set_title(f'{axis_name.upper()} Axis - Position & Velocity')
        ax.grid(True, alpha=0.3)
        
        # 位置
        ax.plot(t, data[f'pos_{axis_name}'], 'C0-', label='Position (m)', linewidth=1.0)
        ax.set_ylabel('Position (m)', color='C0', fontsize=10)
        ax.tick_params(axis='y', labelcolor='C0')
        
        # 速度 (右轴)
        ax_v = ax.twinx()
        ax_v.plot(t, data[f'vel_{axis_name}'], 'C1-', label='Velocity (m/s)', linewidth=1.0)
        ax_v.set_ylabel('Velocity (m/s)', color='C1', fontsize=10)
        ax_v.tick_params(axis='y', labelcolor='C1')
        
        # 命令速度 (第三轴)
        ax_cmd = ax.twinx()
        ax_cmd.spines['right'].set_position(('outward', 60))
        ax_cmd.plot(t, data[f'cmd_{axis_name}'], 'C2--', label='Cmd (m/s)', linewidth=1.0)
        ax_cmd.set_ylabel('Cmd (m/s)', color='C2', fontsize=10)
        ax_cmd.tick_params(axis='y', labelcolor='C2')
        
        ax.set_xlabel('Time (s)', fontsize=10)
        
        # 图例
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax_v.get_legend_handles_labels()
        lines3, labels3 = ax_cmd.get_legend_handles_labels()
        ax.legend(lines1 + lines2 + lines3, labels1 + labels2 + labels3, 
                 loc='upper left', fontsize=9)
    
    # ─────────────────────────
    # 图2：XY 防摇分析
    # ─────────────────────────
    fig2, axes2 = plt.subplots(2, 1, figsize=(14, 8), constrained_layout=True)
    fig2.suptitle(f'Offline Analysis: Anti-Sway - {Path(filename).stem}', 
                  fontsize=14, fontweight='bold')
    
    for idx, direction in enumerate(['x', 'y']):
        ax = axes2[idx]
        ax.set_title(f'{direction.upper()} Direction - Swing Angle & Angular Velocity')
        ax.grid(True, alpha=0.3)
        
        # 摆角
        ax.plot(t, data[f'sw_{direction}'], 'C3-', label='Swing Angle (rad)', 
               linewidth=1.0)
        ax.set_ylabel('Swing Angle (rad)', color='C3', fontsize=10)
        ax.tick_params(axis='y', labelcolor='C3')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3, linewidth=1.0)
        
        # 角速度 (右轴)
        ax_aw = ax.twinx()
        ax_aw.plot(t, data[f'aw_{direction}'], 'C4--', label='Angular Velocity (rad/s)', 
                  linewidth=1.0)
        ax_aw.set_ylabel('Angular Velocity (rad/s)', color='C4', fontsize=10)
        ax_aw.tick_params(axis='y', labelcolor='C4')
        
        ax.set_xlabel('Time (s)', fontsize=10)
        
        # 图例
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax_aw.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=9)
    
    # ─────────────────────────
    # 保存图表
    # ─────────────────────────
    output_fig1 = filename.replace('.json', '_motion_analysis.png')
    output_fig2 = filename.replace('.json', '_sway_analysis.png')
    
    fig1.savefig(output_fig1, dpi=150, bbox_inches='tight')
    fig2.savefig(output_fig2, dpi=150, bbox_inches='tight')
    
    print(f"✓ 运动控制图表已保存: {output_fig1}")
    print(f"✓ 防摇分析图表已保存: {output_fig2}")
    
    # ─────────────────────────
    # 统计信息
    # ─────────────────────────
    print("\n" + "="*70)
    print("数据统计信息")
    print("="*70)
    
    for axis_name in ['x', 'y', 'z']:
        pos = np.array(data[f'pos_{axis_name}'])
        vel = np.array(data[f'vel_{axis_name}'])
        cmd = np.array(data[f'cmd_{axis_name}'])
        
        print(f"\n{axis_name.upper()} 轴:")
        print(f"  位置:   范围 [{pos.min():.4f}, {pos.max():.4f}] m, "
              f"均值 {pos.mean():.4f} m, 标准差 {pos.std():.4f} m")
        print(f"  速度:   范围 [{vel.min():.4f}, {vel.max():.4f}] m/s, "
              f"均值 {vel.mean():.4f} m/s, 标准差 {vel.std():.4f} m/s")
        print(f"  命令:   范围 [{cmd.min():.4f}, {cmd.max():.4f}] m/s")
    
    print("\n" + "="*70)
    print("防摇信息")
    print("="*70)
    
    for direction in ['x', 'y']:
        swing = np.array(data[f'sw_{direction}'])
        aw = np.array(data[f'aw_{direction}'])
        
        print(f"\n{direction.upper()} 方向:")
        print(f"  摆角:     范围 [{swing.min():.6f}, {swing.max():.6f}] rad, "
              f"最大振幅 {max(abs(swing.min()), abs(swing.max())):.6f} rad")
        print(f"  角速度:   范围 [{aw.min():.6f}, {aw.max():.6f}] rad/s")
    
    # 非阻塞显示图表，允许继续交互
    plt.show(block=False)
    plt.pause(0.1)


def list_available_files():
    """列出可用的录制文件"""
    files = sorted(glob.glob('data/recorded_data_*.json'))
    if not files:
        print("未找到任何录制数据文件 (data/recorded_data_*.json)")
        return None
    
    print("\n可用的录制文件:")
    for i, f in enumerate(files):
        size = Path(f).stat().st_size / 1024  # KB
        print(f"  [{i}] {f} ({size:.1f} KB)")
    
    return files


def validate_and_load_file(filename_or_index, available_files=None):
    """验证并加载文件
    
    支持三种输入形式：
    1. 数字索引：选择可用文件列表中的文件 (e.g., "0")
    2. 完整路径：直接加载指定文件 (e.g., "data/recorded_data_*.json")
    3. 文件名：查找匹配的文件 (e.g., "recorded_data_*.json")
    """
    input_str = filename_or_index.strip()
    
    # 检查是否为数字索引
    if input_str.isdigit():
        if available_files is None:
            print("❌ 没有可用文件列表")
            return None
        
        idx = int(input_str)
        if 0 <= idx < len(available_files):
            return available_files[idx]
        else:
            print(f"❌ 索引超出范围 [0-{len(available_files)-1}]")
            return None
    
    # 检查完整路径是否存在
    if Path(input_str).exists():
        if input_str.endswith('.json'):
            return input_str
        else:
            print(f"❌ 不是 JSON 文件: {input_str}")
            return None
    
    # 尝试在 data 目录中查找
    data_path = Path('data') / input_str
    if data_path.exists():
        return str(data_path)
    
    # 尝试模糊匹配
    matches = glob.glob(f"data/*{input_str}*")
    if matches:
        if len(matches) == 1:
            return matches[0]
        else:
            print(f"❌ 找到多个匹配文件:")
            for m in matches:
                print(f"    {m}")
            return None
    
    print(f"❌ 找不到文件: {input_str}")
    return None


def interactive_mode():
    """交互模式"""
    print("\n" + "="*70)
    print("进入交互分析模式")
    print("="*70)
    print("命令说明:")
    print("  - 输入数字: 从列表中选择文件 (例: 0, 1, 2)")
    print("  - 输入文件名: 分析指定文件")
    print("  - 输入 'ls':  列出所有可用文件")
    print("  - 输入 'quit' 或 'exit': 退出程序")
    print("="*70 + "\n")
    
    available_files = list_available_files()
    
    while True:
        try:
            user_input = input("\n请输入文件编号或文件名 > ").strip()
            
            # 检查退出命令
            if user_input.lower() in ['quit', 'exit', 'q']:
                print("✓ 程序已退出")
                break
            
            # 检查列表命令
            if user_input.lower() == 'ls':
                list_available_files()
                continue
            
            # 验证并加载文件
            if not user_input:
                continue
            
            filename = validate_and_load_file(user_input, available_files)
            if filename:
                load_and_plot_data(filename)
                # 重新刷新可用文件列表（可能有新文件）
                available_files = list_available_files()
        
        except KeyboardInterrupt:
            print("\n\n✓ 程序已退出")
            break
        except Exception as e:
            print(f"❌ 错误: {e}")


if __name__ == '__main__':
    if len(sys.argv) > 1:
        # 指定文件
        filename = sys.argv[1]
        if not Path(filename).exists():
            print(f"❌ 文件不存在: {filename}")
            sys.exit(1)
        load_and_plot_data(filename)
        interactive_mode()
    else:
        # 自动查找最新的录制文件
        files = list_available_files()
        if files:
            print(f"\n使用最新文件: {files[-1]}")
            load_and_plot_data(files[-1])
            interactive_mode()
        else:
            sys.exit(1)
