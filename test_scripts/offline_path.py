import snap7
from snap7.util import set_int, set_real
import pandas as pd
import struct
import time

class PLCTrajectoryTool:
    def __init__(self, ip, rack=0, slot=1, db_number=10):
        self.client = snap7.client.Client()
        self.ip = ip
        self.rack = rack
        self.slot = slot
        self.db_num = db_number
        self.batch_id = 1
        
    def connect(self):
        try:
            self.client.connect(self.ip, self.rack, self.slot)
            print(f"成功连接至 PLC: {self.ip}")
        except Exception as e:
            print(f"连接失败: {e}")

    def send_batch(self, points_df):
        """
        将 DataFrame 中的点发送至 PLC
        points_df 包含列: x, y, z, vx, vy, vz
        """
        num_points = len(points_df)
        if num_points > 100:
            print("警告：单批次最多支持 100 个点，将截断。")
            num_points = 100
            points_df = points_df.head(100)

        # 1. 创建缓冲区 (14字节偏移 + 100个点*24字节 = 2414 字节左右)
        # 我们根据实际点数动态或固定长度打包，这里采用覆盖模式
        buffer = bytearray(2500) 

        # 2. 写入 BatchID (Offset 8) 和 MovePointNum (Offset 10)
        set_int(buffer, 8, self.batch_id)
        
        set_int(buffer, 10, num_points)

        # 3. 写入点数据 (Offset 14 开始)
        current_offset = 14
        for _, row in points_df.iterrows():
            # 依次写入 x, y, z, vx, vy, vz (每个都是 REAL/4字节)
            set_real(buffer, current_offset,      float(row['x']))
            set_real(buffer, current_offset + 4,  float(row['y']))
            set_real(buffer, current_offset + 8,  float(row['z']))
            set_real(buffer, current_offset + 12, float(row['vx']))
            set_real(buffer, current_offset + 16, float(row['vy']))
            set_real(buffer, current_offset + 20, float(row['vz']))
            current_offset += 24

        set_int(buffer, 2414, self.batch_id)
        # 4. 一次性写入 PLC DB块
        # 写数据长度：从偏移8开始，到最后一个点结束
        write_size = current_offset - 8
        self.client.db_write(self.db_num, 8, buffer[8:2500])
        
        print(f"已发送批次 {self.batch_id}，包含 {num_points} 个轨迹点。")
        self.batch_id += 1

def main():
    # 参数配置
    PLC_IP = '192.168.111.240'  # 请修改为你的 PLC IP
    CSV_FILE = './test_scripts/trajectory_2001.csv' # 你的文件名
    
    tool = PLCTrajectoryTool(PLC_IP)
    tool.connect()

    if not tool.client.get_connected():
        return

    # 加载数据
    try:
        df = pd.read_csv(CSV_FILE)
    except Exception as e:
        print(f"读取 CSV 失败: {e}")
        return

    total_rows = len(df)
    current_row = 0

    print("\n--- 终端交互工具已就绪 ---")
    print("输入 'n' 发送下一批 (100点)")
    print("输入 'r' 重置数据指针")
    print("输入 'q' 退出")

    while True:
        cmd = input("\n请输入命令: ").strip().lower()
        
        if cmd == 'n':
            if current_row >= total_rows:
                print("所有数据已发送完毕！")
                continue
            
            # 取出下一批数据
            end_row = min(current_row + 100, total_rows)
            batch_df = df.iloc[current_row:end_row]
            
            tool.send_batch(batch_df)
            current_row = end_row
            print(f"进度: {current_row}/{total_rows}")
            
        elif cmd == 'r':
            current_row = 0
            tool.batch_id = 1
            print("进度已重置。")
            
        elif cmd == 'q':
            tool.client.disconnect()
            print("已断开连接，退出。")
            break
        else:
            print("未知命令。")

if __name__ == "__main__":
    main()