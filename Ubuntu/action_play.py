import serial
import time
import struct

# ================= 配置参数 =================
PORT = '/dev/ttyACM0'
BAUDRATE = 1000000 
ACTION_FILENAME = 'action_list.txt'
# 播放间隔 (秒) - 建议与录制时的 RECORD_INTERVAL 保持一致
# 这个数越大，两帧之间的时间间隔就越低，播放的速度就越慢
PLAY_INTERVAL = 0.02

# 帧头定义
HEADER_1 = 0xAA
HEADER_2 = 0x55

def calculate_checksum(data_bytes):
    """
    计算简单的 8 位累加和校验 (Checksum-8)
    只对传入的数据段进行累加，取低 8 位 (即与 0xFF 进行与操作)
    """
    return sum(data_bytes) & 0xFF

def main():
    try:
        # 打开串口
        ser = serial.Serial()
        ser.port = PORT
        ser.baudrate = BAUDRATE
        ser.timeout = 1
        
        # 保持 DTR/RTS 拉低
        ser.dtr = False
        ser.rts = False
        ser.open()
        ser.dtr = False
        ser.rts = False
        
        print(f"✅ 成功连接串口 {PORT}。")
        
        # 读取动作文件
        print(f"📂 正在加载动作文件 {ACTION_FILENAME} ...")
        with open(ACTION_FILENAME, 'r') as f:
            lines = f.readlines()
            
        if not lines:
            print("❌ 文件为空，没有可播放的动作！")
            return

        print(f"▶️ 加载成功，共 {len(lines)} 帧动作。开始播放...")
        
        # 循环遍历每一行数据
        for index, line in enumerate(lines):
            # 去除首尾空白符，按空格分割，并转换为整数列表
            # 例如将 "1024 2048 512" 变成 [1024, 2048, 512]
            str_values = line.strip().split(' ')
            
            # 过滤掉空行或格式不正确的行
            if len(str_values) != 12:
                continue
                
            positions = [int(val) for val in str_values]
            
            # 1. 打包有效数据 (12个 uint16_t, 小端模式 -> 24 bytes)
            # *positions 会把列表解包成独立的参数传给 struct
            payload = struct.pack('<12H', *positions)
            
            # 2. 计算校验码 (基于 payload 的 24 个字节)
            checksum = calculate_checksum(payload)
            
            # 3. 组装完整的数据帧 (Header1 + Header2 + Payload + Checksum)
            # 使用 struct 打包帧头和校验码 ('BB' 代表 2个 unsigned char)
            frame = struct.pack('BB', HEADER_1, HEADER_2) + payload + struct.pack('B', checksum)
            
            # 4. 发送到串口
            ser.write(frame)
            
            # 打印进度 (可选，如果觉得太刷屏可以注释掉)
            print(f"发送第 {index + 1}/{len(lines)} 帧 | 数据: {positions[0]} {positions[1]}...")
            
            # 5. 延时，控制播放速度
            time.sleep(PLAY_INTERVAL)

        print("\n🎉 动作播放完毕！")

    except FileNotFoundError:
        print(f"❌ 找不到文件 '{ACTION_FILENAME}'，请确认是否先运行了录制脚本。")
    except serial.SerialException as e:
        print(f"❌ 串口异常: {e}")
    except KeyboardInterrupt:
        print("\n🛑 检测到 Ctrl+C，已中止播放。")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已安全关闭。")

if __name__ == '__main__':
    main()