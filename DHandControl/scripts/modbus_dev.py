from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time

class DexHandControl:
    """
    机器人手控制类，仅支持Modbus通信方式
    提供两个底层函数：move_fingers（控制电缸组）和 move_palms（控制舵机组）
    """
    
    def __init__(self, port='COM3', baudrate=9600, parity='N', stopbits=1, bytesize=8):
        """
        初始化Modbus连接参数
        :param port: 串口号 (Windows: 'COM3', Linux: '/dev/ttyUSB0')
        :param baudrate: 波特率 (默认9600)
        :param parity: 校验位 (默认'N' - 无校验)
        :param stopbits: 停止位 (默认1)
        :param bytesize: 数据位 (默认8)
        """
        self.client = ModbusClient(
            method='rtu',
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize
        )
        self.last_status = 0
    
    def connect(self):
        """连接Modbus设备"""
        return self.client.connect()
    
    def disconnect(self):
        """断开Modbus连接"""
        self.client.close()
    
    def _send_command(self, cmd, params=None):
        """
        发送Modbus命令
        :param cmd: 命令ID (1=单个设备控制, 2=组控, 3=清除错误)
        :param params: 参数字典 {寄存器地址: 值}
        """
        if not self.connect():
            print("Modbus连接失败")
            return False
        
        try:
            # 设置命令寄存器
            self.client.write_register(address=0, value=cmd, unit=1)
            
            # 设置参数
            if params:
                for addr, value in params.items():
                    self.client.write_register(address=addr, value=value, unit=1)
            
            # 读取状态反馈
            result = self.client.read_holding_registers(address=5, count=1, unit=1)
            if result:
                self.last_status = result.registers[0]
                return True
            return False
        except Exception as e:
            print(f"Modbus错误: {e}")
            return False
        finally:
            self.disconnect()
    
    def move_fingers(self, id_list, pos_list):
        """
        同步控制多个电缸运动（手指）
        :param id_list: 电缸ID列表 [1,2,...]
        :param pos_list: 目标位置列表 [p1,p2,...]
        :return: 是否成功执行
        """
        # 设置组控参数
        group_size = len(id_list)
        params = {
            1: 0,  # 设备类型: 电缸
            6: group_size  # 组控数量
        }
        
        # 写入组控数据
        for i, (id_val, pos_val) in enumerate(zip(id_list, pos_list)):
            params[10 + i*2] = id_val
            params[11 + i*2] = pos_val
        
        return self._send_command(2, params)
    
    def move_palms(self, id_list, pos_list, time_list):
        """
        同步控制多个舵机运动（手掌）
        :param id_list: 舵机ID列表 [1,2,...]
        :param pos_list: 目标位置列表 [p1,p2,...]
        :param time_list: 运动时间列表 [t1,t2,...](毫秒)
        :return: 是否成功执行
        """
        # 设置组控参数
        group_size = len(id_list)
        params = {
            1: 1,  # 设备类型: 舵机
            6: group_size  # 组控数量
        }
        
        # 写入组控数据
        for i, (id_val, pos_val, time_val) in enumerate(zip(id_list, pos_list, time_list)):
            params[10 + i*3] = id_val
            params[11 + i*3] = pos_val
            params[12 + i*3] = time_val
        
        return self._send_command(2, params)
    
    def clear_error(self, servo_id):
        """
        清除舵机错误状态
        :param servo_id: 舵机ID
        :return: 是否成功执行
        """
        params = {
            2: servo_id  # 设备ID
        }
        return self._send_command(3, params)
    
    def get_status(self):
        """
        获取最后的状态码
        :return: 状态码
        """
        return self.last_status
    
    def decode_status(self, status=None):
        """
        解码状态寄存器值
        :param status: 状态码 (默认为最后的状态码)
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
            0xE3: "组控数量超限",
            0xF0: "清除错误成功"
        }
        
        # 基础状态解码
        base_status = status & 0xF0
        if base_status in status_map:
            base_msg = status_map[base_status]
            detail = status & 0x0F
            return f"{base_msg} (详情: 0x{detail:X})"
        
        return f"未知状态: 0x{status:X}"

# 使用示例
if __name__ == "__main__":
    # 创建Modbus控制对象
    hand = DexHandControl(port='COM3', baudrate=9600)
    
    # 控制电缸组（手指）
    success = hand.move_fingers([1, 2, 3], [500, 600, 700])
    if success:
        print("电缸组控成功:", hand.decode_status())
    else:
        print("电缸组控失败")
    
    time.sleep(1)
    
    # 控制舵机组（手掌）
    success = hand.move_palms([1, 2, 3], [1000, 1500, 2000], [1000, 1500, 2000])
    if success:
        print("舵机组控成功:", hand.decode_status())
    else:
        print("舵机组控失败")
    
    time.sleep(1)
    
    # 清除错误
    success = hand.clear_error(1)
    if success:
        print("清除错误成功:", hand.decode_status())
    else:
        print("清除错误失败")
