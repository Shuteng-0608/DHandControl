from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient as ModbusClient
import time
import threading


class DexHandControl:
    """
    机器人手控制类 - Modbus协议
    Modbus/RS485配置：115200波特率，8位数据位，偶校验，1位停止位
    """

    def __init__(self, port='COM3', baudrate=115200, parity='E', stopbits=1, bytesize=8, timeout=3):
        """
        初始化Modbus连接参数
        :param port: 串口号 (Windows: 'COM3', Linux: '/dev/ttyUSB0')
        :param baudrate: Modbus/RS485波特率 115200
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
        self.persistent_connection = False
        self.transaction_lock = threading.Lock()

    def _client_connected(self):
        connected = getattr(self.client, "connected", False)
        return connected() if callable(connected) else bool(connected)

    def connect(self):
        """连接Modbus设备"""
        try:
            if self._client_connected():
                return True
            return self.client.connect()
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开Modbus连接"""
        self.client.close()

    def start_persistent_connection(self):
        """开启持久Modbus连接，用于高频遥操作"""
        with self.transaction_lock:
            self.persistent_connection = True
            if not self.connect():
                self.persistent_connection = False
                print("持久Modbus连接失败")
                return False
            return True

    def stop_persistent_connection(self):
        """关闭持久Modbus连接"""
        with self.transaction_lock:
            self.persistent_connection = False
            self.disconnect()

    def _ensure_ok(self, response, operation_name):
        """验证Modbus响应，失败时抛出带上下文的异常"""
        if response is None:
            raise RuntimeError(f"{operation_name} 无响应")
        if response.isError():
            raise RuntimeError(f"{operation_name} 返回Modbus错误: {response}")
        return response

    def _write_register_checked(self, address, value, operation_name=None):
        """写单个寄存器并验证响应"""
        operation_name = operation_name or f"写寄存器 {address}"
        result = self.client.write_register(address=address, value=value, device_id=1)
        self._ensure_ok(result, operation_name)

    def _read_register_checked(self, address, operation_name=None):
        """读单个保持寄存器并验证响应"""
        operation_name = operation_name or f"读寄存器 {address}"
        result = self.client.read_holding_registers(address=address, count=1, device_id=1)
        result = self._ensure_ok(result, operation_name)
        if not hasattr(result, "registers") or len(result.registers) < 1:
            raise RuntimeError(f"{operation_name} 响应缺少寄存器数据")
        return result.registers[0]

    def _send_command(self, cmd, params=None):
        """
        发送Modbus命令（修正顺序）
        :param cmd: 命令ID (1=单个设备控制, 2=组控, 3=清除错误)
        :param params: 参数字典 {寄存器地址: 值}
        :return: 是否成功执行
        """
        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return False

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return False

            try:
                # 先设置参数，最后设置命令寄存器触发执行
                if params:
                    for addr, value in params.items():
                        result = self.client.write_register(address=addr, value=value, device_id=1)
                        self._ensure_ok(result, f"写参数寄存器 {addr}")

                # 最后设置命令寄存器触发执行
                result = self.client.write_register(address=0, value=cmd, device_id=1)
                self._ensure_ok(result, "写命令寄存器")

                # 等待命令执行完成
                time.sleep(0.1)

                # 读取状态反馈
                result = self.client.read_holding_registers(address=5, count=1, device_id=1)
                result = self._ensure_ok(result, "读取状态寄存器")
                if not hasattr(result, "registers") or len(result.registers) < 1:
                    raise RuntimeError("读取状态寄存器响应缺少寄存器数据")
                self.last_status = result.registers[0]
                return True
            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return False
            finally:
                if owns_connection:
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

    def move_hand(self, finger_ids=None, finger_positions=None,
                  palm_ids=None, palm_positions=None, palm_times=None):
        """
        组合控制手指电缸和手掌舵机
        :param finger_ids: 电缸ID列表
        :param finger_positions: 电缸目标位置列表 (0-2000)
        :param palm_ids: 舵机ID列表
        :param palm_positions: 舵机目标位置列表 (0-1000)
        :param palm_times: 舵机运动时间列表(ms)
        :return: 是否成功执行
        """
        finger_ids = [] if finger_ids is None else list(finger_ids)
        finger_positions = [] if finger_positions is None else list(finger_positions)
        palm_ids = [] if palm_ids is None else list(palm_ids)
        palm_positions = [] if palm_positions is None else list(palm_positions)
        palm_times = [] if palm_times is None else list(palm_times)

        if len(finger_ids) != len(finger_positions):
            print("错误: 手指ID列表和位置列表长度不一致")
            return False

        if len(palm_ids) != len(palm_positions) or len(palm_ids) != len(palm_times):
            print("错误: 手掌ID列表、位置列表和时间列表长度不一致")
            return False

        finger_count = len(finger_ids)
        palm_count = len(palm_ids)

        if finger_count == 0 and palm_count == 0:
            print("错误: 组合控制至少需要一个手指或手掌设备")
            return False

        if finger_count > 5:
            print("错误: 手指组控数量不能超过5")
            return False

        if palm_count > 5:
            print("错误: 手掌组控数量不能超过5")
            return False

        for id_val in finger_ids + palm_ids:
            if not isinstance(id_val, int) or id_val < 0 or id_val > 255:
                print(f"错误: 设备ID {id_val} 超出范围 (0-255)")
                return False

        for pos in finger_positions:
            if not isinstance(pos, int) or pos < 0 or pos > 2000:
                print(f"错误: 手指位置值 {pos} 超出范围 (0-2000)")
                return False

        for pos in palm_positions:
            if not isinstance(pos, int) or pos < 0 or pos > 1000:
                print(f"错误: 手掌位置值 {pos} 超出范围 (0-1000)")
                return False

        for time_val in palm_times:
            if not isinstance(time_val, int) or time_val < 0 or time_val > 65535:
                print(f"错误: 手掌运动时间 {time_val} 超出范围 (0-65535)")
                return False

        params = {
            20: finger_count,  # REG_HAND_FINGER_COUNT
            31: palm_count  # REG_HAND_PALM_COUNT
        }

        for i, (id_val, pos_val) in enumerate(zip(finger_ids, finger_positions)):
            params[21 + i * 2] = id_val
            params[21 + i * 2 + 1] = pos_val

        for i, (id_val, pos_val, time_val) in enumerate(zip(palm_ids, palm_positions, palm_times)):
            params[32 + i * 3] = id_val
            params[32 + i * 3 + 1] = pos_val
            params[32 + i * 3 + 2] = time_val

        return self._send_command(4, params)

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
        if dev_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return False

        params = {
            1: dev_type,  # 设备类型
            2: dev_id  # 设备ID
        }

        success = self._send_command(3, params)
        if not success:
            return False

        if self.last_status == 0xF0:
            return True

        print("清除错误失败:", self.decode_status())
        return False

    def read_device_id(self, device_type, query_id):
        """
        读取设备ID
        :param device_type: 设备类型 (0=电缸, 1=舵机)
        :param query_id: 查询ID
        :return: 读取到的ID，失败返回None
        """
        if device_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return None

        if not isinstance(query_id, int) or query_id < 1 or query_id > 253:
            print(f"错误: 查询ID {query_id} 超出范围 (1-253)")
            return None

        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return None

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return None

            try:
                self._write_register_checked(1, device_type, "写设备类型")
                self._write_register_checked(2, query_id, "写查询ID")
                self._write_register_checked(0, 0x05, "写读取ID命令")

                time.sleep(0.1)

                self.last_status = self._read_register_checked(5, "读取状态寄存器")
                if self.last_status != 0x91:
                    print("读取设备ID失败:", self.decode_status())
                    return None

                return self._read_register_checked(8, "读取ID结果寄存器")
            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return None
            finally:
                if owns_connection:
                    self.disconnect()

    def set_device_id(self, device_type, old_id, new_id, save=True):
        """
        修改设备ID
        :param device_type: 设备类型 (0=电缸, 1=舵机)
        :param old_id: 当前ID
        :param new_id: 新ID
        :param save: 是否请求保存配置
        :return: 是否成功
        """
        if device_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return False

        if not isinstance(old_id, int) or old_id < 1 or old_id > 253:
            print(f"错误: 当前ID {old_id} 超出范围 (1-253)")
            return False

        if not isinstance(new_id, int) or new_id < 1 or new_id > 253:
            print(f"错误: 新ID {new_id} 超出范围 (1-253)")
            return False

        if old_id == new_id:
            print("错误: 当前ID和新ID相同")
            return False

        with self.transaction_lock:
            owns_connection = not self.persistent_connection

            if self.persistent_connection and not self._client_connected():
                print("持久Modbus连接已断开")
                return False

            if owns_connection and not self.connect():
                print("Modbus连接失败")
                return False

            try:
                self._write_register_checked(1, device_type, "写设备类型")
                self._write_register_checked(2, old_id, "写当前ID")
                self._write_register_checked(7, new_id, "写新ID")
                self._write_register_checked(9, 1 if save else 0, "写ID保存标志")
                self._write_register_checked(0, 0x06, "写设置ID命令")

                time.sleep(0.2)

                self.last_status = self._read_register_checked(5, "读取状态寄存器")
                if self.last_status != 0x92:
                    print("设置设备ID失败:", self.decode_status())
                    return False

                result_id = self._read_register_checked(8, "读取ID结果寄存器")
                if result_id != new_id:
                    print(f"设置设备ID失败: 回读ID {result_id} != 新ID {new_id}")
                    return False

                return True
            except Exception as e:
                print(f"Modbus通信错误: {e}")
                return False
            finally:
                if owns_connection:
                    self.disconnect()

    def read_palm_id(self, query_id):
        """读取手掌舵机ID"""
        return self.read_device_id(1, query_id)

    def set_palm_id(self, old_id, new_id, save=True):
        """修改手掌舵机ID"""
        return self.set_device_id(1, old_id, new_id, save)

    def read_finger_id(self, query_id):
        """读取手指电缸ID"""
        return self.read_device_id(0, query_id)

    def set_finger_id(self, old_id, new_id, save=True):
        """修改手指电缸ID"""
        return self.set_device_id(0, old_id, new_id, save)

    def scan_device_ids(self, device_type, start_id=1, end_id=30):
        """
        扫描设备ID
        :param device_type: 设备类型 (0=电缸, 1=舵机)
        :param start_id: 起始ID
        :param end_id: 结束ID
        :return: 响应的ID列表
        """
        if device_type not in (0, 1):
            print("错误: 设备类型必须为0(电缸)或1(舵机)")
            return []

        if (not isinstance(start_id, int) or not isinstance(end_id, int) or
                start_id < 1 or end_id > 253 or start_id > end_id):
            print("错误: 扫描范围必须在1..253内，且起始ID不能大于结束ID")
            return []

        found = []
        for query_id in range(start_id, end_id + 1):
            device_id = self.read_device_id(device_type, query_id)
            if device_id is not None:
                found.append(device_id)
        return found

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
            0x90: "组合手部控制命令已下发",
            0x91: "设备ID读取成功",
            0x92: "设备ID设置成功",
            0xA0: "电缸控制成功",
            0xB0: "舵机控制成功",
            0xC0: "电缸组控成功",
            0xD0: "舵机组控成功",
            0xE0: "无效命令错误",
            0xE1: "固件校验错误: 无效设备类型",
            0xE2: "控制失败错误",
            0xE3: "固件校验错误: 无效组控数量",
            0xE4: "组控失败错误",
            0xE5: "清除错误暂不支持",
            0xE6: "固件校验错误: 无效寄存器地址",
            0xE7: "固件校验错误: 不支持的Modbus功能码",
            0xE8: "固件校验错误: 无效组合手指数量",
            0xE9: "固件校验错误: 无效组合手掌数量",
            0xEA: "固件校验错误: 组合控制寄存器范围无效",
            0xEB: "固件校验错误: 组合控制为空",
            0xEC: "设备ID读取失败",
            0xED: "设备ID设置失败",
            0xEE: "固件校验错误: 无效设备ID",
            0xEF: "设备ID操作暂不支持",
            0xF0: "清除错误成功"
        }

        if status in status_map:
            return status_map[status]

        base_status = status & 0xF0
        if base_status in status_map:
            detail = status & 0x0F
            return f"{status_map[base_status]} (详情: 0x{detail:X})"

        return f"未知状态: 0x{status:X}"
    
    def demo():
        pass

    def thumb_index(self):
        self.move_fingers([1, 2, 3, 4, 5], [640, 1200, 20, 20, 20])
    
    def thumb_mid(self):
        self.move_fingers([1, 2, 3, 4, 5], [1200, 20, 1750, 20, 20])
        self.move_palms([1, 2, 3], [700, 600, 520], [1000, 1000, 1000])
    
    def rock(self):
        self.move_fingers([1, 2, 3, 4, 5], [1200, 20, 1750, 1600, 20])
        self.move_palms([1, 2, 3], [700, 600, 520], [1000, 1000, 1000])

    def boxing(self):
        self.move_fingers([1, 2, 3, 4, 5], [20, 1950, 1950, 1950, 1950])
        time.sleep(0.5)
        self.move_fingers([1], [600])
    
    def one(self):
        self.move_fingers([1, 2, 3, 4, 5], [1000, 20, 1950, 1950, 1950])
    
    def two(self):
        self.move_fingers([1, 2, 3, 4, 5], [1200, 20, 20, 1950, 1950])



    
    def palm_free(self):
        self.move_palms([1, 2, 3], [753, 500, 500], [1000, 1000, 1000])
    
    def finger_free(self):
        self.move_fingers([1, 2, 3, 4, 5], [20, 20, 20, 20, 20])

    def free(self):
        self.palm_free()
        self.finger_free()
        


# 使用示例
if __name__ == "__main__":
    # 创建Modbus控制对象
    hand = DexHandControl(
        port='/dev/ttyUSB0',  # 根据实际串口修改
        baudrate=115200,  # Modbus/RS485 baudrate
        parity='E',  # 偶校验
        stopbits=1,
        bytesize=8,
        timeout=3
    )

    try:
        
        hand.free()
        time.sleep(1)

        hand.one()
        time.sleep(1)
        hand.two()
        time.sleep(1)
        hand.rock()
        time.sleep(1)
        hand.boxing()
        time.sleep(1)
        hand.thumb_index()
        time.sleep(1)
        hand.thumb_mid()

        time.sleep(0.5)
        hand.free()

        # time.sleep(1)

        # hand.move_fingers([1,2,3,4,5], [20,20,20,20,20])
        # time.sleep(2)
        # for i in range(2):

        #     hand.move_fingers([2,3,4,5], [1950,1950,1950,1950])
        #     time.sleep(0.4)

        #     hand.move_fingers([1], [600])
        #     time.sleep(1)

        #     # 示例: 同步控制多个电缸（手指）
        #     # hand.move_fingers([1,2,3,4,5], [500,800,800,800,800])
        #     # time.sleep(2)
        #     hand.move_fingers([1,2,3,4,5], [20,20,20,20,20])
        #     time.sleep(1)

        # 示例: 同步控制多个舵机（手掌）
        # hand.move_palms([2],[500],[2000])

        # hand.move_fingers([3],[1750])
        # hand.move_fingers([1], [1200])
        # hand.move_palms([3],[520],[1000])
        # hand.move_palms([1],[700],[1000])
        # hand.move_palms([2],[600],[1000])




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
        success = hand.clear_error(dev_id=0)
        print("状态:", hand.decode_status())

    except Exception as e:
        print(f"执行错误: {e}")
