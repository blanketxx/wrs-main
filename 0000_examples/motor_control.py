import can
import struct
import time

class CANFDController:
    def __init__(self, channel='can0', bitrate=5000000):
        """
        初始化 CANFD 控制器
        """ 
        self.channel = channel
        self.bitrate = bitrate
        self.bus = self.init_canfd_interface()

    def init_canfd_interface(self):
        """
        初始化 CANFD 设备
        """
        try:
            bus = can.interface.Bus(bustype='socketcan', channel=self.channel, fd=True, bitrate=self.bitrate)
            print("CANFD 设备初始化成功。")
            return bus
        except Exception as e:
            print("CANFD 设备初始化失败：", e)
            return None

    def send_can_message(self, can_id, data):
        """
        发送 CANFD 消息
        """
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            print(f"发送 CAN 消息: ID={hex(can_id)}, 数据={data}")
        except can.CanError:
            print("CAN 消息发送失败。")

    def set_work_mode(self, mode):
        """
        设置电机工作模式
        """
        mode_map = {"position": 0x03, "current": 0x01, "speed": 0x02}
        if mode not in mode_map:
            raise ValueError("无效模式！请选择 'position', 'current', 或 'speed'。")
        self.send_can_message(0x01, [0x02, 0x30, mode_map[mode]])

    def enable_motor(self, enable=True):
        """
        使能/禁用驱动器
        """
        value = 0x01 if enable else 0x00
        self.send_can_message(0x01, [0x02, 0x0A, value])

    def send_position_command(self, angle):
        """
        发送位置指令（单位 0.0001°）
        """
        pos_value = int(angle * 10000)
        data = list(struct.pack('<I', pos_value))
        self.send_can_message(0x201, data)

    def send_speed_command(self, speed):
        """
        发送速度指令（单位：0.01°/s）
        """
        speed_value = int(speed * 100)
        data = list(struct.pack('<I', speed_value))
        self.send_can_message(0x301, data)

    def send_current_command(self, current):
        """
        发送电流指令（单位：mA）
        """
        current_value = int(current)
        data = list(struct.pack('<I', current_value))
        self.send_can_message(0x401, data)

    def get_position(self, timeout=5.0):
        """
        监听 CAN 消息，获取当前角度（单位：度）。
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=timeout)
            if msg and msg.arbitration_id == 0x501:
                # 当前位置数据在 D8~D11，即第 8 到第 11 字节
                position = struct.unpack('<I', bytes(msg.data[8:12]))[0] / 10000.0
                print(f"当前位置: {position:.4f}°")
                return position
        print("超时未收到位置数据")
        return None

    def get_speed(self, timeout=5.0):
        """
        监听 CAN 消息，获取当前速度（单位：度/秒）。
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=timeout)
            if msg and msg.arbitration_id == 0x501:
                # 当前速度数据在 D4~D7，即第 4 到第 7 字节
                speed = struct.unpack('<i', bytes(msg.data[4:8]))[0] / 50.0
                print(f"当前速度: {speed:.2f}°/s")
                return speed
        print("超时未收到速度数据")
        return None

    def get_current(self, timeout=5.0):
        """
        监听 CAN 消息，获取当前电流（单位：mA）。
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=timeout)
            if msg and msg.arbitration_id == 0x501:
                # 当前电流数据在 D0~D3，即第 0 到第 3 字节
                current = struct.unpack('<i', bytes(msg.data[:4]))[0]
                print(f"当前电流: {current} mA")
                return current
        print("超时未收到电流数据")
        return None

    def main(self):
        if not self.bus:
            return

        while True:
            cmd = input("输入命令 (mode/enable/disable/pos/speed/current/pos_now/speed_now/current_now/exit)：").strip()
            if cmd == "mode":
                mode = input("选择模式 (position/speed/current)：").strip()
                self.set_work_mode(mode)
            elif cmd == "enable":
                self.enable_motor(True)
            elif cmd == "disable":
                self.enable_motor(False)
            elif cmd == "pos":
                angle = float(input("输入目标角度 (°)："))
                self.send_position_command(angle)
            elif cmd == "speed":
                speed = float(input("输入目标速度 (°/s)："))
                self.send_speed_command(speed)
            elif cmd == "current":
                current = float(input("输入目标电流 (mA)："))
                self.send_current_command(current)
            elif cmd == "pos_now":
                self.get_position()
            elif cmd == "speed_now":
                self.get_speed()
            elif cmd == "current_now":
                self.get_current()
            elif cmd == "exit":
                print("退出程序。")
                break
            else:
                print("无效命令！")

        self.bus.shutdown()

if __name__ == "__main__":
    controller = CANFDController()
    controller.main()