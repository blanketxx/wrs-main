import can
import struct
import time


def init_canfd_interface(channel='can0', bitrate=5000000):
    """
    初始化 CANFD 设备
    """
    try:
        bus = can.interface.Bus(bustype='socketcan', channel=channel, fd=True, bitrate=bitrate)
        print("CANFD 设备初始化成功。")
        return bus
    except Exception as e:
        print("CANFD 设备初始化失败：", e)
        return None


def send_can_message(bus, can_id, data):
    """
    发送 CANFD 消息
    """
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"发送 CAN 消息: ID={hex(can_id)}, 数据={data}")
    except can.CanError:
        print("CAN 消息发送失败。")


def set_work_mode(bus, mode):
    """
    设置电机工作模式
    """
    mode_map = {"position": 0x03, "current": 0x01, "speed": 0x02}
    if mode not in mode_map:
        raise ValueError("无效模式！请选择 'position', 'current', 或 'speed'。")
    send_can_message(bus, 0x01, [0x02, 0x30, mode_map[mode]])


def enable_motor(bus, enable=True):
    """
    使能/禁用驱动器
    """
    value = 0x01 if enable else 0x00
    send_can_message(bus, 0x01, [0x02, 0x0A, value])


def send_position_command(bus, angle):
    """
    发送位置指令（单位 0.0001°）
    """
    pos_value = int(angle * 10000)
    data = list(struct.pack('<I', pos_value))
    send_can_message(bus, 0x201, data)


def send_speed_command(bus, speed):
    """
    发送速度指令（单位：0.01°/s）
    """
    speed_value = int(speed * 100)
    data = list(struct.pack('<I', speed_value))
    send_can_message(bus, 0x202, data)


def send_current_command(bus, current):
    """
    发送电流指令（单位：mA）
    """
    current_value = int(current)
    data = list(struct.pack('<I', current_value))
    send_can_message(bus, 0x203, data)


def receive_response(bus, expected_id, timeout=1.0):
    """
    接收 CAN 消息
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout=timeout)
        if msg and msg.arbitration_id == expected_id:
            print(f"收到响应: ID={hex(msg.arbitration_id)}, 数据={list(msg.data)}")
            return msg
    print(f"超时未收到 ID={hex(expected_id)} 的响应。")
    return None


def main():
    bus = init_canfd_interface()
    if not bus:
        return

    while True:
        cmd = input("输入命令 (mode/enable/pos/speed/current/exit)：").strip()
        if cmd == "mode":
            mode = input("选择模式 (position/speed/current)：").strip()
            set_work_mode(bus, mode)
        elif cmd == "enable":
            enable_motor(bus, True)
        elif cmd == "disable":
            enable_motor(bus, False)
        elif cmd == "pos":
            angle = float(input("输入目标角度 (°)："))
            send_position_command(bus, angle)
        elif cmd == "speed":
            speed = float(input("输入目标速度 (°/s)："))
            send_speed_command(bus, speed)
        elif cmd == "current":
            current = float(input("输入目标电流 (mA)："))
            send_current_command(bus, current)
        elif cmd == "exit":
            print("退出程序。")
            break
        else:
            print("无效命令！")

    bus.shutdown()


if __name__ == "__main__":
    main()
