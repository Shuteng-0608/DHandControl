from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient as ModbusClient
import time


class DexHandControl:
    """
    机器人手控制类 - Modbus协议
    根据原理图配置：921600波特率，偶校验
    """

    def __init__(self, port='COM3', baudrate=921600, parity='E', stopbits=1, bytesize=8, timeout=3):
        """
        初始化Modbus连接参数（根据原理图修正）
        :param port: 串口号 (Windows: 'COM3', Linux: '/dev/ttyUSB0')
        :param baudrate: 波特率 921600（根据原理图）
        :param parity: 校验位 'E' - 偶校验（Modbus标准）
        :param stopbits: 停止位 1
        :param bytesize: 数据位 8
        :param timeout: 超时时间 3秒
        """
        self.client = ModbusClient(
            port=port,
            framer=FramerType.RTU,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout
        )
        self.last_status = 0

    def connect(self):
        """连接Modbus设备"""
        try:
            return self.client.connect()
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开Modbus连接"""
        self.client.close()

    def _send_command(self, cmd, params=None):
        """
        发送Modbus命令（修正顺序）
        :param cmd: 命令ID (1=单个设备控制, 2=组控, 3=清除错误)
        :param params: 参数字典 {寄存器地址: 值}
        :return: 是否成功执行
        """
        if not self.connect():
            print("Modbus连接失败")
            return False

        try:
            # 先设置参数，最后设置命令寄存器触发执行
            if params:
                for addr, value in params.items():
                    self.client.write_register(address=addr, value=value, device_id=1)

            # 最后设置命令寄存器触发执行
            self.client.write_register(address=0, value=cmd, device_id=1)

            # 等待命令执行完成
            time.sleep(0.1)

            # 读取状态反馈
            result = self.client.read_holding_registers(address=5, count=1, device_id=1)
            if result and not result.isError():
                self.last_status = result.registers[0]
                return True
            return False
        except Exception as e:
            print(f"Modbus通信错误: {e}")
            return False
        finally:
            self.disconnect()

    def move_fingers(self, id_list, pos_list):
        """
        同步控制多个电缸运动（手指）
        :param id_list: 电缸ID列表 [1,2,...]
        :param pos_list: 目标位置列表 [p1,p2,...] (0-2000)
        :return: 是否成功执行
        """
        if len(id_list) != len(pos_list):
            print("错误: ID列表和位置列表长度不一致")
            return False

        # 验证位置范围
        for pos in pos_list:
            if pos < 0 or pos > 2000:
                print(f"错误: 位置值 {pos} 超出范围 (0-2000)")
                return False

        group_size = len(id_list)
        if group_size > 5:
            print("错误: 组控数量不能超过5")
            return False

        params = {
            1: 0,  # 设备类型: 电缸
            6: group_size  # 组控数量
        }

        # 写入组控数据
        for i, (id_val, pos_val) in enumerate(zip(id_list, pos_list)):
            params[10 + i * 2] = id_val
            params[11 + i * 2] = pos_val

        return self._send_command(2, params)

    def move_palms(self, id_list, pos_list, time_list):
        """
        同步控制多个舵机运动（手掌）
        :param id_list: 舵机ID列表 [1,2,...]
        :param pos_list: 目标位置列表 [p1,p2,...] (0-2000)
        :param time_list: 运动时间列表 [t1,t2,...](毫秒)
        :return: 是否成功执行
        """
        if len(id_list) != len(pos_list) or len(id_list) != len(time_list):
            print("错误: ID列表、位置列表和时间列表长度不一致")
            return False

        # 验证位置范围
        for pos in pos_list:
            if pos < 0 or pos > 1000:
                print(f"错误: 位置值 {pos} 超出范围 (0-1000)")
                return False

        group_size = len(id_list)
        if group_size > 5:
            print("错误: 组控数量不能超过5")
            return False

        params = {
            1: 1,  # 设备类型: 舵机
            6: group_size  # 组控数量
        }

        # 写入组控数据
        for i, (id_val, pos_val, time_val) in enumerate(zip(id_list, pos_list, time_list)):
            params[10 + i * 3] = id_val
            params[11 + i * 3] = pos_val
            params[12 + i * 3] = time_val

        return self._send_command(2, params)

    def single_control(self, dev_type, dev_id, position, time_val=1000):
        """
        单个设备控制
        :param dev_type: 设备类型 (0=电缸, 1=舵机)
        :param dev_id: 设备ID
        :param position: 目标位置 (0-1000)
        :param time_val: 执行时间(ms)，仅对舵机有效
        :return: 是否成功执行
        """
        if position < 0 or position > 2000:
            print(f"错误: 位置值 {position} 超出范围 (0-2000)")
            return False

        params = {
            1: dev_type,  # 设备类型
            2: dev_id,  # 设备ID
            3: position,  # 位置
            4: time_val  # 执行时间
        }

        return self._send_command(1, params)

    def clear_error(self, dev_id, dev_type=0):
        """
        清除设备错误状态
        :param dev_type: 设备类型 (0=电缸, 1=舵机)
        :param dev_id: 设备ID
        :return: 是否成功执行
        """
        params = {
            1: dev_type,  # 设备类型
            2: dev_id  # 设备ID
        }
        return self._send_command(3, params)

    def get_status(self):
        """获取最后的状态码"""
        return self.last_status

    def decode_status(self, status=None):
        """
        解码状态寄存器值
        :param status: 状态码
        :return: 状态描述
        """
        if status is None:
            status = self.last_status

        status_map = {
            0xA0: "电缸控制成功",
            0xB0: "舵机控制成功",
            0xC0: "电缸组控成功",
            0xD0: "舵机组控成功",
            0xE0: "无效命令错误",
            0xE1: "位置超限错误",
            0xE2: "控制失败错误",
            0xE3: "组控数量超限",
            0xE4: "组控失败错误",
            0xE5: "清除错误失败",
            0xF0: "清除错误成功"
        }

        base_status = status & 0xF0
        if base_status in status_map:
            detail = status & 0x0F
            return f"{status_map[base_status]} (详情: 0x{detail:X})"

        return f"未知状态: 0x{status:X}"


# 使用示例
if __name__ == "__main__":
    # 创建Modbus控制对象（根据原理图配置）
    hand = DexHandControl(
        port='COM4',  # 根据实际串口修改
        baudrate=921600,  # 根据原理图使用921600
        parity='E',  # 偶校验
        stopbits=1,
        bytesize=8,
        timeout=3
    )

    try:

        # for i in range(10):
        #     # 示例1: 控制电缸组（手指）
        #     # print("控制电缸组...")
        #     hand.move_fingers([1, 2], [1000, 800])
        #     print("状态:", hand.decode_status())
        #
        #     # 示例2: 控制舵机组（手掌）
        #     # print("控制舵机组...")
        #     hand.move_palms(id_list=[1, 2], pos_list=[200, 500], time_list=[500, 500] )
        #     print("状态:", hand.decode_status())
        #
        #     time.sleep(1)
        #     hand.move_fingers([1, 2], [400, 1600])
        #     print("状态:", hand.decode_status())
        #     hand.move_palms(id_list=[1, 2], pos_list=[500, 100], time_list=[1000, 500])
        #     print("状态:", hand.decode_status())
        #
        # time.sleep(1)

        # # 示例3: 单个设备控制
        # print("单个舵机控制...")
        # for i in range(10):
        #     hand.single_control(dev_type=1, dev_id=1, position=800, time_val=1000)
        #     time.sleep(0.5)
        #     hand.single_control(dev_type=1, dev_id=2, position=800, time_val=500)
        #     time.sleep(0.5)
        #     hand.single_control(dev_type=1, dev_id=1, position=200, time_val=1000)
        #     time.sleep(0.5)
        #     hand.single_control(dev_type=1, dev_id=2, position=200, time_val=500)
        #     time.sleep(0.5)
        # # print("状态:", hand.decode_status())
        #
        # print("单个电缸控制...")
        # for i in range(10):
        #     hand.single_control(dev_type=0, dev_id=1, position=800, time_val=1000)
        #     time.sleep(0.5)
        #     hand.single_control(dev_type=0, dev_id=2, position=800, time_val=500)
        #     time.sleep(0.5)
        #     hand.single_control(dev_type=0, dev_id=1, position=1200, time_val=1000)
        #     time.sleep(0.5)
        #     hand.single_control(dev_type=0, dev_id=2, position=1200, time_val=500)
        #     time.sleep(0.5)
        #
        # time.sleep(2)
        #
        # # 示例4: 清除错误
        print("清除电缸错误...")
        success = hand.clear_error(dev_id=1)
        print("状态:", hand.decode_status())

    except Exception as e:
        print(f"执行错误: {e}")