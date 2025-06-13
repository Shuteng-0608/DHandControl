#!/usr/bin/env python

import rospy
from aiui.srv import DH5SetPosition, DH5SetPositionResponse
import threading
import serial
import struct
import random
import numpy as np
import time

class DH5ModbusAPI:
    SUCCESS = 0
    ERROR_CONNECTION_FAILED = 1
    ERROR_INVALID_RESPONSE = 2
    ERROR_CRC_CHECK_FAILED = 3
    ERROR_INVALID_COMMAND = 4

    def __init__(self, port='COM6', modbus_id=1, baud_rate=115200, stop_bits=1, parity='N'):
        self.port = port
        self.modbus_id = modbus_id
        self.baud_rate = baud_rate
        self.stop_bits = stop_bits
        self.parity = parity
        self.serial_connection = None

    def open_connection(self):
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                stopbits=self.stop_bits,
                parity=self.parity,
                timeout=1
            )
            if self.serial_connection.is_open:
                print("Serial connection opened successfully")
                return self.SUCCESS
        except Exception as e:
            return f"Failed to open serial connection: {str(e)}"

    def close_connection(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            return self.SUCCESS

    def send_modbus_command(self, function_code, register_address, data=None, data_length=None):
        if not self.serial_connection or not self.serial_connection.is_open:
            return self.ERROR_CONNECTION_FAILED

        try:
            if function_code == 0x03:  # Read Holding Registers
                message = self._build_request(function_code, register_address, data_length=data_length or 1)
            elif function_code == 0x06:  # Write Single Register
                message = self._build_request(function_code, register_address, value=data)
            elif function_code == 0x10:  # Write Multiple Registers
                message = self._build_request(function_code, register_address, values=data, data_length=data_length)
            else:
                return self.ERROR_INVALID_COMMAND

            self.serial_connection.write(message)
            response = self.serial_connection.read(256)
            return self._parse_response(response, function_code)
        except Exception as e:
            return f"Error: {str(e)}"

    def _build_request(self, function_code, register_address, data_length=1, value=None, values=None):
        request = bytearray()
        request.append(self.modbus_id)
        request.append(function_code)
        request += struct.pack('>H', register_address)

        if function_code == 0x03:  # Read Holding Registers
            request += struct.pack('>H', data_length)
        elif function_code == 0x06:  # Write Single Register
            request += struct.pack('>H', value)
        elif function_code == 0x10:  # Write Multiple Registers
            register_count = data_length if data_length else len(values)
            request += struct.pack('>H', register_count)  # Number of registers
            request.append(register_count * 2)  # Byte count (register_count * 2 bytes per register)
            for val in values:
                request += struct.pack('>H', val)

        crc = self._calculate_crc(request)
        request += struct.pack('<H', crc)
        return request

    @staticmethod
    def _calculate_crc(data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def _parse_response(self, response, function_code):
        if len(response) < 5:
            # raise ValueError("Incomplete response received")
            return self.ERROR_INVALID_RESPONSE

        device_id, func_code, *payload, crc_low, crc_high = response
        if func_code != function_code:
            # raise ValueError(f"Unexpected function code: {func_code}")
            return self.ERROR_INVALID_RESPONSE

        crc_received = (crc_high << 8) | crc_low
        crc_calculated = self._calculate_crc(response[:-2])
        if crc_received != crc_calculated:
            return self.ERROR_CRC_CHECK_FAILED

        if function_code == 0x03:
            data = payload[1:]  # Skip byte count
            return [struct.unpack('>H', bytes(data[i:i + 2]))[0] for i in range(0, len(data), 2)]
        elif function_code in [0x06, 0x10]:
            return self.SUCCESS

    # -------------------- Configuration Functions --------------------
    def set_config(self, modbus_id=None, baud_rate=None, stop_bits=None, parity=None):
        if modbus_id:
            self.modbus_id = modbus_id
        if baud_rate:
            self.baud_rate = baud_rate
        if stop_bits:
            self.stop_bits = stop_bits
        if parity:
            self.parity = parity

    # -------------------- API Methods --------------------

    def set_uart_config(self, modbus_id=None, baud_rate=None, stop_bits=None, parity=None):
        uart_registers = [modbus_id, baud_rate, stop_bits, parity]
        self.send_modbus_command(function_code=0x10, register_address=0x0302, data=uart_registers,
                                 data_length=len(uart_registers))

    def set_save_param(self, flag=1):
        self.send_modbus_command(function_code=0x06, register_address=0x0300, data=flag)

    def initialize(self, mode):
        """
        Initialize all 6 axes with a specific mode.
        Modes:
          - 0b01: Close
          - 0b10: Open
          - 0b11: Find total stroke
        """
        if mode not in [0b01, 0b10, 0b11]:
            return self.ERROR_INVALID_COMMAND

        data = 0
        for axis in range(6):
            data |= (mode << (axis * 2))  # Set all axes to the given mode
        return self.send_modbus_command(function_code=0x06, register_address=0x0100, data=data)

    def initialize_axis(self, axis, mode):
        """
        Initialize a specific axis with a specific mode.
        Modes:
          - 0b01: Close
          - 0b10: Open
          - 0b11: Find total stroke
        """
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        if mode not in [0b01, 0b10, 0b11]:
            return self.ERROR_INVALID_COMMAND

        init_status = 0
        # Set 2 bits for the specified axis
        init_status &= ~(0b11 << ((axis - 1) * 2))  # Clear the bits for the axis
        init_status |= (mode << ((axis - 1) * 2))  # Set the mode bits for the axis
        return self.send_modbus_command(function_code=0x06, register_address=0x0100, data=init_status)

    def check_initialization(self):
        """
        Check the initialization status of all 6 axes.
        Returns:
            A dictionary where each key represents an axis (axis_F1, etc.) and the value is the initialization status:
              - "not initialized": Axis has no initialization (00)
              - "initialized": Axis initialization successful (01)
              - "initializing": Axis is currently initializing (10)
        """
        response = self.send_modbus_command(function_code=0x03, register_address=0x0200, data_length=1)
        if isinstance(response, list) and len(response) > 0:
            init_status = response[0]
            status = {}
            for axis in range(6):
                axis_status = (init_status >> (axis * 2)) & 0b11  # Extract 2 bits for each axis
                if axis_status == 0b01:
                    status[f"axis_F{axis + 1}"] = "initialized"
                elif axis_status == 0b10:
                    status[f"axis_F{axis + 1}"] = "initializing"
                else:
                    status[f"axis_F{axis + 1}"] = "not initialized"
            return status
        return self.ERROR_INVALID_RESPONSE

    def set_axis_position(self, axis, position):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0101 + (axis - 1)
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=position)

    def set_all_position(self, position_list, axis_list=[1, 2, 3, 4, 5, 6]):
        """
        运动到指定位置
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        register_address = 0x0101
        if self.port == '/dev/ttyUSB0':
            # Right hand
            position_list = self.clamp_list(position_list, position_limits_right)
        if self.port == '/dev/ttyUSB1':
            # Left hand
            position_list = self.err_comp(position_list)
            position_list = self.clamp_list(position_list, position_limits_left)

        return self.send_modbus_command(function_code=0x10,
                                        register_address=register_address,
                                        data=position_list,
                                        data_length=len(axis_list))

    def set_axis_speed(self, axis, speed):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x010D + (axis - 1)
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=speed)

    def set_all_speed(self, axis_list, speed_list):
        """
        运动段的最大速度
          - speed_list: 1 ~ 100，百分比
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        register_address = 0x010D
        return self.send_modbus_command(function_code=0x10,
                                        register_address=register_address,
                                        data=speed_list,
                                        data_length=len(axis_list))

    def set_axis_acc(self, axis, acc):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0113 + (axis - 1) * 0x10
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=acc)

    def set_all_acc(self, axis_list, acc_list):
        """
        运动段的加减速
          - acc_list: 1 ~ 100，百分比
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        register_address = 0x0113
        return self.send_modbus_command(function_code=0x10,
                                        register_address=register_address,
                                        data=acc_list,
                                        data_length=len(axis_list))

    def set_axis_force(self, axis, force):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0107 + (axis - 1) * 0x10
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=force)

    def set_all_force(self, axis_list, force_list):
        """
        开环力控下，以额定电流百分比设定推压段输出力
          - 开环力控下：20 ~ 100，百分比
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        register_address = 0x0107
        return self.send_modbus_command(function_code=0x10,
                                        register_address=register_address,
                                        data=force_list,
                                        data_length=len(axis_list))

    def set_all(self, position_list, axis_list=None, force_list=None, speed_list=None, acc_list=None):
        """
        设置所有关节的目标位置、速度、力、加速度
        :param axis_list: 关节列表
        :param position_list: 目标位置列表
        :param speed_list: 速度列表
        :param force_list: 力列表
        :param acc_list: 加速度列表
        :return:
        """
        if axis_list is None or [0]:
            axis_list = [1, 2, 3, 4, 5, 6]
        if force_list is None or [0]:
            force_list = [100, 100, 100, 100, 100, 100]
        if speed_list is None or [0]:
            speed_list = [50, 50, 50, 50, 50, 50]
        if acc_list is None or [0]:
            acc_list = [100, 100, 100, 100, 100, 100]
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        position_register_address = 0x0101
        force_register_address = 0x0107
        speed_register_address = 0x010D
        acc_register_address = 0x0113
        if self.port == '/dev/ttyUSB0':
            # Right hand
            position_list = self.clamp_list(position_list, position_limits_right)
        if self.port == '/dev/ttyUSB1':
            # Left hand
            position_list = self.clamp_list(self.err_comp(position_list), position_limits_left)
        complete_list = position_list + force_list + speed_list + acc_list
        return self.send_modbus_command(function_code=0x10,
                                        register_address=position_register_address,
                                        data=complete_list,
                                        data_length=len(complete_list))

    def get_all_feedback(self):
        register_address = 0x0201
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=24)

    def parse_axis_state(self, response_data):
        """
        参数:
            response_data: 包含24个数据的列表
        返回:
            包含四个子列表的字典:
            - 'state': 运行状态 [0:运动中, 1:到达位置, 2:堵转]
            - 'position': 当前位置
            - 'speed': 运行速度
            - 'current': 当前电流
        """
        # if len(response_data) != 24:
        #     raise ValueError("响应数据长度必须为24")

        for i in range(len(response_data)):
            # print(i)
            response_data[i] = self.to_signed_16bit(response_data[i])

        state = response_data[0:6]  # 第1-6个: 运行状态
        position = response_data[6:12]  # 第7-12个: 当前位置
        speed = response_data[12:18]  # 第13-18个: 运行速度
        current = response_data[18:24]  # 第19-24个: 当前电流

        return {
            'state': state,
            'position': position,
            'speed': speed,
            'current': current
        }

    def get_all_state(self):
        """
        state
          - [0]: 运动中
          - [1]: 到达位置
          - [2]: 堵转
        """
        register_address = 0x0201
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=6)

    def get_axis_position(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0207 + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_all_position(self):
        """
        state
          - [0]: 运动中
          - [1]: 到达位置
          - [2]: 堵转
        """
        register_address = 0x0207
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=6)

    def get_axis_speed(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x020D + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_all_speed(self):
        register_address = 0x020D
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=6)

    def get_axis_current(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0213 + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_all_current(self):
        register_address = 0x0213
        return self.send_modbus_command(function_code=0x03, register_address=register_address, data_length=6)

    def get_cur_faults(self):
        return self.send_modbus_command(function_code=0x03, register_address=0x021F, data_length=1)

    def get_history_faults(self):
        return self.send_modbus_command(function_code=0x03, register_address=0x0B00, data_length=0x3F)

    def reset_faults(self):
        return self.send_modbus_command(function_code=0x06, register_address=0x0501, data=1)

    def restart_system(self):
        return self.send_modbus_command(function_code=0x06, register_address=0x0503, data=1)

    def clamp_value(self, value, min_val, max_val):
        """限制值在 [min_val, max_val] 之间"""
        return max(min_val, min(max_val, value))

    def clamp_list(self, values, limits):
        """限制列表的每个值在对应位置的范围内"""
        return [self.clamp_value(values[i], limits[i][0], limits[i][1]) for i in range(6)]

    def to_signed_16bit(self, value):
        """手动转换 16 位有符号数"""
        if value >= 0x8000:  # 32768（0x8000）及以上表示负数
            return value - 0x10000  # 转换为负数
        return value

    def err_comp(self, right, gain_err=None):
        """误差补偿"""
        if gain_err is None:
            gain_err = [4, 0, 24, -30, 40, -43]
        gain_left = [0, 0, 0, 0, 0, 0]
        for i in range(len(right)):
            gain_left[i] = gain_err[i] + right[i]
        return gain_left

    def perform(self, gesture):
        """
        gesture_list: ["ONE", "YE", "OK", "FIVE", "ROCK"]
        """
        if gesture not in gesture_list:
            return self.ERROR_INVALID_COMMAND
        # if self.port == '/dev/ttyUSB1':
        #     self.set_all_position(self.err_comp(gesture_list["FIVE"]))
        #     time.sleep(0.5)
        #     self.set_all_position(self.err_comp(gesture_list[gesture]))
        # if self.port == '/dev/ttyUSB0':
        self.set_all_position(gesture_list["FIVE"])
        time.sleep(0.5)
        self.set_all_position(gesture_list[gesture])

    def demo(self):
        for j in range(1, 100):
            gesture_name = random.choice(list(gesture_list.keys()))
            self.perform(gesture_name)
            print(f"Perform {j}: {gesture_name}")
            time.sleep(0.5)
    
    def gripper(self, state):
        if state == "open":
            # == OPEN == #
            result = self.set_all([930, 1770, 1707, 1730, 1730, 980], speed_list=[30, 30, 30, 30, 30, 30])
        elif state == "close":
            # == CLOSE == #
            result = self.set_all([300, 500, 500, 500, 500, 400], speed_list=[100, 30, 30, 30, 30, 30])
        else:
            result = 2
        return result


def handle_set_position(req):
    # 根据请求中的hand_type决定使用哪个机械手
    if len(req.position_list) != 6 and req.hand_mode == 'hand':
        rospy.logerr("DH hand requires exactly 6 positions")
        return DH5SetPositionResponse(-1)
    if req.gripper_state not in ['open', 'close'] and req.hand_mode == 'gripper':
        rospy.logerr(f"Invalid gripper state: {req.gripper_state}")
        return DH5SetPositionResponse(-1)
    if req.hand_type == 'right' and req.hand_mode == 'hand': # RIGHT hand
        rospy.loginfo("Setting positions for RIGHT hand")
        result = api_r.set_all(req.position_list, 
                               axis_list=req.axis_list,
                               force_list=req.force_list,
                               speed_list=req.speed_list,
                               acc_list=req.acc_list)
    elif req.hand_type == 'left' and req.hand_mode == 'hand': # LEFT hand
        rospy.loginfo("Setting positions for LEFT hand")
        result = api_l.set_all(req.position_list, 
                               axis_list=req.axis_list,
                               force_list=req.force_list,
                               speed_list=req.speed_list,
                               acc_list=req.acc_list)
    elif req.hand_type == 'right' and req.hand_mode == 'gripper': # RIGHT hand
        rospy.loginfo(f"Setting RIGHT gripper-hand: {req.gripper_state}")
        result = api_r.gripper(req.gripper_state)
    elif req.hand_type == 'left' and req.hand_mode == 'gripper': # LEFT hand
        rospy.loginfo(f"Setting LEFT gripper-hand: {req.gripper_state}")
        result = api_l.gripper(req.gripper_state)
    else:
        rospy.logerr(f"Invalid hand_type | hand_mode: {req.hand_type} | {req.hand_mode}")
        return DH5SetPositionResponse(-1)
    
    if result == DH5ModbusAPI.SUCCESS:
        rospy.loginfo("Position set successfully")
        return DH5SetPositionResponse(0)
    else:
        rospy.logerr(f"Position set failed with error: {result}")
        return DH5SetPositionResponse(-1)

def shutdown_hook():
    rospy.loginfo("Shutting down, closing connections")
    api_r.close_connection()
    api_l.close_connection()

if __name__ == '__main__':
    """
    sudo chmod 666 /dev/ttyUSB0
    sudo chmod 666 /dev/ttyUSB1
    """
    # 初始化ROS节点
    rospy.init_node('dh5_hand_controller')
    
    rospy.loginfo("Starting DH5 Hand Controller Node")

    # RIGHT   [930, 1771, 1707, 1731, 1731, 981]
    gesture_list = {
        "ONE": [30, 1770, 30, 30, 30, 825],
        "YE": [30, 1770, 1707, 30, 30, 200],
        "OK": [354, 1080, 1707, 1730, 1730, 418],
        "GOOD": [930, 10, 30, 30, 30, 980],
        "FIVE": [930, 1770, 1707, 1730, 1730, 980],
        "ROCK": [930, 1770, 30, 30, 1730, 980]
    }

    #### Left Hand Initialization #### ttyUSB1
    """
    axis_F1     30 - 934      大拇指左右转向
    axis_F2     10 - 1771     食指
    axis_F3     30 - 1731     中指
    axis_F4     30 - 1701     无名指 
    axis_F5     10 - 1771     小拇指
    axis_F6     30 - 938      大拇指上下转向
    """
    api_l = DH5ModbusAPI(port='/dev/ttyUSB1', baud_rate=115200)
    rospy.loginfo(api_l.open_connection())
    rospy.loginfo(api_l.initialize(0b10))
    rospy.loginfo(api_l.check_initialization())
    position_limits_left = [
        [30, 934],
        [10, 1771],
        [30, 1731],
        [30, 1701],
        [10, 1771],
        [30, 938],
    ]

    #### Right Hand Initialization #### ttyUSB0
    """
    axis_F1     30 - 930     大拇指左右转向
    axis_F2     10 - 1771    食指
    axis_F3     30 - 1707    中指
    axis_F4     30 - 1731    无名指 
    axis_F5     30 - 1731    小拇指
    axis_F6     30 - 981     大拇指上下转向
    """
    api_r = DH5ModbusAPI(port='/dev/ttyUSB0', baud_rate=115200)
    rospy.loginfo(api_r.open_connection())
    rospy.loginfo(api_r.initialize(0b10))
    rospy.loginfo(api_r.check_initialization())
    position_limits_right = [
        [30, 930],
        [10, 1771],
        [30, 1707],
        [30, 1731],
        [30, 1731],
        [30, 981],
    ]

    """
    error_compensation:
        RIGHT   [930, 1771, 1707, 1731, 1731, 981]
        LEFT    [934, 1771, 1731, 1701, 1771, 938]
        ERR_GAIN = LEFT - RIGHT  [+4, 0, +24, -30, +40, -43]
    """
    # err_gain = [4, 0, 24, -30, 40, -43]
    rospy.on_shutdown(shutdown_hook)


    # r_state = api_r.get_all_feedback()
    # r_parsed_data = api_r.parse_axis_state(r_state)
    # rospy.loginfo("RIGHT 运行状态:", r_parsed_data['state'])
    # rospy.loginfo("RIGHT 当前位置:", r_parsed_data['position'])
    # rospy.loginfo("RIGHT 运行速度:", r_parsed_data['speed'])
    # rospy.loginfo("RIGHT 当前电流:", r_parsed_data['current'])

    # l_state = api_l.get_all_feedback()
    # l_parsed_data = api_l.parse_axis_state(l_state)
    # rospy.loginfo("LEFT 运行状态:", l_parsed_data['state'])
    # rospy.loginfo("LEFT 当前位置:", l_parsed_data['position'])
    # rospy.loginfo("LEFT 运行速度:", l_parsed_data['speed'])
    # rospy.loginfo("LEFT 当前电流:", l_parsed_data['current'])
    
    
    # 创建ROS服务
    service = rospy.Service('/dh5/set_all_position', DH5SetPosition, handle_set_position)
    rospy.loginfo("SetAllPosition service ready")
    
    # 进入事件循环
    rospy.spin()
