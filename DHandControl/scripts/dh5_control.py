import time
import threading
import serial
import struct


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

    def _calculate_crc(self, data):
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

    def set_save_param(self, flag = 1):
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
            A dictionary where each key represents an axis (axis_F1, axis_F2, etc.) and the value is the initialization status:
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

    def set_all_positions(self, axis_list, position_list):
        """
        同时设置所有关节的目标位置
        :param axis_list: 关节列表
        :param position_list: 目标位置列表
        :return:
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        register_address = 0x0101
        return self.send_modbus_command(function_code=0x10,
                                        register_address=register_address,
                                        data=position_list,
                                        data_length=len(axis_list))

    def set_axis_speed(self, axis, speed):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x010D + (axis - 1)
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=speed)

    def set_all_speeds(self, axis_list, speed_list):
        """
        同时设置所有关节的速度
        :param axis_list: 关节列表
        :param speed_list: 速度列表
        :return:
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        register_address = 0x010D
        return self.send_modbus_command(function_code=0x10,
                                        register_address=register_address,
                                        data=speed_list,
                                        data_length=len(axis_list))

    def set_axis_force(self, axis, force):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0107 + (axis - 1) * 0x10
        return self.send_modbus_command(function_code=0x06, register_address=register_address, data=force)

    def set_all_forces(self, axis_list, force_list):
        """
        同时设置所有关节的力
        :param axis_list: 关节列表
        :param force_list: 力列表
        :return:
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        register_address = 0x0107
        return self.send_modbus_command(function_code=0x10,
                                        register_address=register_address,
                                        data=force_list,
                                        data_length=len(axis_list))

    def set_all(self, axis_list, position_list, speed_list, force_list):
        """
        设置所有关节的目标位置、速度、力
        :param axis_list: 关节列表
        :param position_list: 目标位置列表
        :param speed_list: 速度列表
        :param force_list: 力列表
        :return:
        """
        for axis in axis_list:
            if axis < 1 or axis > 6:
                return self.ERROR_INVALID_COMMAND
        position_register_address = 0x0101
        speed_register_address = 0x010D
        force_register_address = 0x0107
        self.send_modbus_command(function_code=0x10,
                                 register_address=speed_register_address,
                                 data=speed_list,
                                 data_length=len(axis_list))
        self.send_modbus_command(function_code=0x10,
                                 register_address=force_register_address,
                                 data=force_list,
                                 data_length=len(axis_list))
        return self.send_modbus_command(function_code=0x10,
                                        register_address=position_register_address,
                                        data=position_list,
                                        data_length=len(axis_list))

    def get_axis_position(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0207 + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_axis_speed(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x020D + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_axis_current(self, axis):
        if axis < 1 or axis > 6:
            return self.ERROR_INVALID_COMMAND
        register_address = 0x0213 + (axis - 1)
        return self.send_modbus_command(function_code=0x03, register_address=register_address)

    def get_cur_faults(self):
        return self.send_modbus_command(function_code=0x03, register_address=0x021F, data_length=1)

    def get_history_faults(self):
        return self.send_modbus_command(function_code=0x03, register_address=0x0B00, data_length=0x3F)

    def reset_faults(self):
        return self.send_modbus_command(function_code=0x06, register_address=0x0501, data=1)

    def restart_system(self):
        return self.send_modbus_command(function_code=0x06, register_address=0x0503, data=1)


def grab():
    api_r.set_all_speeds([1, 2, 3, 4, 5, 6], [30, 30, 30, 30, 30, 30])
    api_l.set_all_speeds([1, 2, 3, 4, 5, 6], [30, 30, 30, 30, 30, 30])
    threads = [
        threading.Thread(target=api_r.set_all_positions, args=([1, 2, 3, 4, 5, 6], [30, 1219, 1135, 1156, 1156, 144])),
        threading.Thread(target=api_l.set_all_positions, args=([1, 2, 3, 4, 5, 6], [30, 1272, 1173, 1128, 1198, 120]))
    ]

    for t in threads:
        t.start()

    for t in threads:
        t.join()

    print("Grab Finish")


if __name__ == '__main__':

    # Left Hand Initialization
    """
    axis_F1     30 - 934     大拇指左右转向
    axis_F2     10 - 1771    食指
    axis_F3     30 - 1731    中指
    axis_F4     30 - 1701    无名指 
    axis_F5     30 - 1771    小拇指
    axis_F6     30 - 938     大拇指上下转向
    """
    api_l = DH5ModbusAPI(port='COM12', baud_rate=115200)
    print(api_l.open_connection())
    print(api_l.initialize(0b10))
    print(api_l.check_initialization())

    # Right Hand Initialization
    """
    axis_F1     30 - 930     大拇指左右转向
    axis_F2     10 - 1771    食指
    axis_F3     30 - 1707    中指
    axis_F4     30 - 1731    无名指 
    axis_F5     30 - 1731    小拇指
    axis_F6     30 - 981     大拇指上下转向
    """
    api_r = DH5ModbusAPI(port='COM9', baud_rate=115200)
    print(api_r.open_connection())
    print(api_r.initialize(0b10))
    print(api_r.check_initialization())

    time.sleep(3)
    grab()

