import serial
import time
import struct
import threading

# ================= 配置参数 =================
PORT = '/dev/ttyACM0'
BAUDRATE = 1000000 
RECORD_FILENAME = 'action_list.txt'
# 记录间隔时间 (秒)。例如 0.02 表示 50Hz，0.05 表示 20Hz
RECORD_INTERVAL = 0.02 

NUM_ELEMENTS = 12
BYTES_PER_ELEMENT = 2
FRAME_SIZE = NUM_ELEMENTS * BYTES_PER_ELEMENT

# ================= 全局变量区 =================
# 存储最新的一帧位姿
latest_positions = [0] * NUM_ELEMENTS
# 线程锁，防止串口写入和文件读取发生冲突
data_lock = threading.Lock()
# 控制程序退出
is_running = True
# 标志位：确认是否收到了至少一帧有效数据
data_valid = False 


def receive_hand_positions(port, baudrate):
    """线程 1：专职负责从串口接收数据并更新全局变量"""
    global latest_positions, is_running, data_valid
    
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baudrate
        ser.timeout = 1
        
        ser.dtr = False
        ser.rts = False
        ser.open()
        ser.dtr = False
        ser.rts = False
        
        print(f"✅ 成功连接串口 {port}。等待 STM32 发送数据...")
        buffer = bytearray()

        while is_running:
            waiting = ser.in_waiting
            if waiting > 0:
                buffer.extend(ser.read(waiting))
                
                while len(buffer) >= FRAME_SIZE:
                    frame_data = buffer[:FRAME_SIZE]
                    del buffer[:FRAME_SIZE]
                    
                    try:
                        unpacked_data = struct.unpack('<12H', frame_data)
                        
                        # ⚠️ 获取锁，安全地更新全局数组
                        with data_lock:
                            latest_positions = list(unpacked_data)
                            
                        # 标记已收到有效数据
                        if not data_valid:
                            data_valid = True
                            print("🎉 收到首帧有效数据，触发动作录制！")
                            
                    except struct.error as e:
                        pass # 忽略解包错误
            else:
                time.sleep(0.002) # 防止 CPU 满载

    except serial.SerialException as e:
        print(f"❌ 串口异常: {e}")
        is_running = False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


def record_action_to_file(filename, interval):
    """线程 2：专职负责按时间间隔读取全局变量并存入文本"""
    global latest_positions, is_running, data_valid
    
    # 死等，直到收到串口发来的第一帧有效数据才开始写文件
    while is_running and not data_valid:
        time.sleep(0.1)
        
    if not is_running:
        return

    print(f"📝 开始以 {interval}s 的间隔将动作写入 {filename} ...")
    
    try:
        # 使用 'w' 模式：每次运行脚本都会覆盖旧文件重新录制
        # 如果想一直追加，可以改成 'a'
        with open(filename, 'w') as f:
            while is_running:
                # ⚠️ 获取锁，安全地复制当前最新位姿
                with data_lock:
                    current_pos = latest_positions.copy()
                
                # 将 12 个数字转换为字符串，以空格分隔（例如: "1024 2048 512 ..."）
                line_str = " ".join([str(val) for val in current_pos])
                
                # 写入文件并换行
                f.write(line_str + "\n")
                
                # 强制刷新缓冲区，防止程序意外崩溃时数据丢失
                f.flush() 
                
                # 等待指定的时间间隔
                time.sleep(interval)
                
    except Exception as e:
        print(f"❌ 写入文件异常: {e}")
        is_running = False


def clear_action_list(filename):
    """
    清空指定的动作列表文件
    """
    try:
        # 以 'w' 模式打开会直接抹除原文件内容
        with open(filename, 'w') as f:
            pass 
        print(f"✨ 已成功清空文件: {filename}")
    except Exception as e:
        print(f"❌ 清空文件时出错: {e}")


def main():
    global is_running

    time.sleep(2) # 等两秒等初始化完成

    clear_action_list(RECORD_FILENAME)
    
    # 1. 创建串口接收线程 (daemon=True 表示主线程退出时它也跟着退出)
    rx_thread = threading.Thread(target=receive_hand_positions, args=(PORT, BAUDRATE), daemon=True)
    
    # 2. 创建文件录制线程
    record_thread = threading.Thread(target=record_action_to_file, args=(RECORD_FILENAME, RECORD_INTERVAL), daemon=True)
    
    # 3. 启动双线程
    rx_thread.start()
    record_thread.start()
    
    # 4. 主线程只负责挂起，并等待用户的 Ctrl+C 打断指令
    try:
        while True:
            time.sleep(1) 
    except KeyboardInterrupt:
        print("\n🛑 检测到 Ctrl+C！正在通知各线程安全停止，请稍候...")
        is_running = False # 修改标志位，通知内部死循环退出

    # 等待子线程安全结束并关闭文件/串口
    rx_thread.join()
    record_thread.join()
    print("✅ 录制结束，程序已安全退出。")

if __name__ == '__main__':
    main()