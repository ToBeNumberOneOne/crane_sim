"""Data Recorder for Crane Simulation

订阅 ZMQ 消息，根据 running 信号自动录制数据。
专门用于后台数据记录，不绘制实时图表。

Usage:
    python record_data.py
"""

import zmq
import json
from datetime import datetime
import time
import signal
import sys

class DataRecorder:
    """仿真数据自动录制器"""
    
    def __init__(self):
        """初始化录制器"""
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.CONFLATE, 1)
        self.sock.setsockopt(zmq.RCVTIMEO, 100)
        self.sock.connect("tcp://localhost:5555")
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # 录制缓冲区
        self.recording_buf = {k: list() for k in
                              ['t', 'timestamp',
                               'pos_x', 'pos_y', 'pos_z',
                               'vel_x', 'vel_y', 'vel_z',
                               'cmd_x', 'cmd_y', 'cmd_z',
                               'sw_x', 'sw_y',
                               'aw_x', 'aw_y',
                               'start', 'running']}
        
        self.is_recording = False
        self.previous_running = False
        self.session_count = 0
        self.t0 = None
        self.total_messages = 0
        self.total_saved = 0
        
        print("=" * 70)
        print("数据录制器已启动")
        print("=" * 70)
        print("等待 ZMQ 消息... (来自 tcp://localhost:5555)")
        print("按 Ctrl+C 退出\n")
    
    def save_recorded_data(self):
        """将记录的数据保存为 JSON 文件"""
        if not self.recording_buf['t']:
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"data/recorded_data_{timestamp}.json"
        
        export_data = {}
        for key in self.recording_buf:
            if self.recording_buf[key]:
                export_data[key] = self.recording_buf[key]
        
        with open(filename, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        num_points = len(self.recording_buf['t'])
        duration = self.recording_buf['t'][-1] - self.recording_buf['t'][0]
        
        print(f"\n{'='*70}")
        print(f"✓ 数据已保存: {filename}")
        print(f"  数据点数:     {num_points}")
        print(f"  时间跨度:     {duration:.3f} 秒")
        print(f"  采样频率:     {num_points/duration:.1f} Hz")
        print(f"{'='*70}\n")
        
        self.total_saved += num_points
        
        # 清空记录缓冲区
        for key in self.recording_buf:
            self.recording_buf[key] = []
    
    def process_message(self, msg):
        """处理单个 ZMQ 消息"""
        self.total_messages += 1
        
        ts = msg['timestamp']
        if self.t0 is None:
            self.t0 = ts
        
        # 解析控制信号
        control_signals = msg.get('control_signals', {})
        start_signal = control_signals.get('start', False)
        running_signal = control_signals.get('running', False)
        
        # ─────────────────────────
        # 记录逻辑：根据 running 信号状态
        # ─────────────────────────
        if running_signal and not self.previous_running:
            # 上升沿：开始记录
            self.is_recording = True
            self.session_count += 1
            print(f"\n[↑] Running 信号上升 → 开始记录 (会话 #{self.session_count})")
            print(f"    时间: {datetime.now().strftime('%H:%M:%S')}")
        
        if self.is_recording:
            # 记录数据
            t_rel = ts - self.t0
            self.recording_buf['t'].append(t_rel)
            self.recording_buf['timestamp'].append(ts)
            self.recording_buf['start'].append(int(start_signal))
            self.recording_buf['running'].append(int(running_signal))
            
            for n in ['x', 'y', 'z']:
                self.recording_buf[f'pos_{n}'].append(msg['pos'][n])
                self.recording_buf[f'vel_{n}'].append(msg['vel'][n])
                self.recording_buf[f'cmd_{n}'].append(msg['cmd_vel'][n])
            
            self.recording_buf['sw_x'].append(msg['swing']['swing_x'])
            self.recording_buf['sw_y'].append(msg['swing']['swing_y'])
            self.recording_buf['aw_x'].append(msg['swing']['ang_vel_x'])
            self.recording_buf['aw_y'].append(msg['swing']['ang_vel_y'])
        
        if not running_signal and self.previous_running:
            # 下降沿：停止记录并保存
            if self.is_recording:
                print(f"[↓] Running 信号下降 → 停止记录")
                print(f"    时间: {datetime.now().strftime('%H:%M:%S')}")
                self.save_recorded_data()
                self.is_recording = False
        
        self.previous_running = running_signal
    
    def run(self):
        """主循环"""
        try:
            while True:
                try:
                    msg = self.sock.recv_json()
                    self.process_message(msg)
                    
                except zmq.Again:
                    # 超时，继续等待
                    sys.stdout.write(f"\r已接收消息: {self.total_messages:6d} | 已保存数据点: {self.total_saved:6d}")
                    sys.stdout.flush()
                    time.sleep(0.01)
                    continue
        
        except KeyboardInterrupt:
            self.shutdown()
    
    def shutdown(self):
        """优雅关闭"""
        print(f"\n\n{'='*70}")
        print("录制器即将关闭...")
        
        # 如果还在录制中，保存数据
        if self.is_recording:
            print("正在保存未完成的录制...")
            self.save_recorded_data()
        
        print(f"\n统计信息:")
        print(f"  总接收消息数:   {self.total_messages}")
        print(f"  总保存数据点:   {self.total_saved}")
        print(f"  记录会话数:     {self.session_count}")
        print(f"{'='*70}\n")
        
        self.sock.close()
        self.ctx.term()
        print("录制器已停止")


if __name__ == '__main__':
    recorder = DataRecorder()
    recorder.run()
