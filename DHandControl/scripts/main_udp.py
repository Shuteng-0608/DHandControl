import json
import socket
import time


class DexHandControl:
    """
    机器人手控制类，封装所有控制方法和预定义手势
    ESP32 IP 列表: 手部控制器默认为 "192.168.4.5"
    """

    def __init__(self, hand_ip="192.168.4.5", pc_ip="192.168.4.10", udp_port=12345):
        """
        初始化控制参数
        :param hand_ip: 手部控制器的IP地址
        :param pc_ip: 本地PC的IP地址
        :param udp_port: UDP通信端口
        """
        self.hand_ip = hand_ip
        self.pc_ip = pc_ip
        self.udp_port = udp_port

    def _send_udp_message(self, message_dict):
        """内部方法：发送JSON格式的UDP消息到手部控制器"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        json_message = json.dumps(message_dict)
        sock.sendto(json_message.encode(), (self.hand_ip, self.udp_port))
        sock.close()

    def servo_move(self, servo_id, position, time_in_ms):
        """
        控制单个舵机运动
        :param servo_id: 舵机ID (1-5)
        :param position: 目标位置 (脉冲宽度, 500-2500)
        :param time_in_ms: 运动时间(毫秒)
        """
        cmd = {
            'Cmd': "ServoMove",
            'ID': servo_id,
            'Pos': position,
            'Time': time_in_ms
        }
        self._send_udp_message(cmd)

    def move_palms(self, id_list, pos_list, time_list):
        """
        同步控制多个舵机运动
        :param id_list: 舵机ID列表 [1,2,...]
        :param pos_list: 目标位置列表 [p1,p2,...]
        :param time_list: 运动时间列表 [t1,t2,...](毫秒)
        """
        cmd = {
            'Cmd': "MovePalms",
            'ID_list': id_list,
            'pos_list': pos_list,
            'time_list': time_list
        }
        self._send_udp_message(cmd)

    def move_fingers(self, id_list, pos_list):
        """
        同步控制多个手指舵机运动
        :param id_list: 手指ID列表 [1,2,...]
                       (1-拇指, 2-食指, 3-中指, 4-无名指, 5-小指)
        :param pos_list: 目标位置列表 [p1,p2,...]
        """
        cmd = {
            'Cmd': "MoveFingers",
            'ID_list': id_list,
            'pos_list': pos_list
        }
        self._send_udp_message(cmd)

    def clear_error(self, servo_id):
        """
        清除舵机错误状态
        :param servo_id: 舵机ID
        """
        cmd = {
            'Cmd': "ClearError",
            'ID': servo_id
        }
        self._send_udp_message(cmd)

    def boxing(self):
        """拳头手势（握拳）"""
        self.move_fingers([2, 3, 4, 5], [2000, 2000, 2000, 2000])
        time.sleep(0.4)
        self.move_fingers([1], [690])
        time.sleep(1.5)
        self.free()

    def index2thumb(self):
        """食指碰拇指（OK手势）"""
        self.move_fingers([1, 2], [600, 1330])
        time.sleep(1.5)
        self.free()

    def middle2thumb(self):
        """中指碰拇指"""
        self.move_fingers([1, 3], [1130, 1700])
        time.sleep(1.5)
        self.free()

    def ring2thumb(self):
        """无名指碰拇指"""
        self.move_palms([2, 1], [380, 530], [1000, 1000])
        self.move_fingers([1, 4], [820, 1360])
        time.sleep(1.5)
        self.free_no_delay()

    def dex_boxing(self):
        """特殊拳击手势"""
        self.move_palms([1, 2], [1000, 131], [1000, 1000])
        self.move_fingers([2, 3, 5], [2000, 2000, 2000])
        time.sleep(0.8)
        self.move_fingers([1, 4], [210, 1060])
        time.sleep(1.5)
        self.free_no_delay()

    def ye(self):
        """"耶"手势（伸出食指和中指）"""
        self.move_fingers([1, 4, 5], [1550, 2000, 2000])
        self.move_palms([3], [426], [1000])
        time.sleep(1.5)
        self.free_no_delay()

    def rock(self):
        """摇滚手势（伸出食指和小指）"""
        self.move_fingers([1, 3, 4], [1050, 2000, 2000])
        time.sleep(1.5)
        self.free_no_delay()

    def one(self):
        """伸出食指（表示数字1）"""
        self.move_fingers([1, 3, 4, 5], [1000, 2000, 2000, 2000])
        time.sleep(1.5)
        self.free()

    def back(self):
        """手掌向后弯曲"""
        self.move_palms([2], [649], [1000])
        time.sleep(1)
        self.free()

    def finger_free(self):
        """手指舒展（张开所有手指）"""
        self.move_fingers([1, 2, 3, 4, 5], [0, 0, 0, 0, 0])
        time.sleep(1)

    def hand_free(self):
        """手掌回中立位"""
        self.move_palms([1, 2, 3], [247, 450, 500], [1000, 1000, 1000])

    def free(self):
        """完全复位（手指舒展+手掌中立）"""
        self.finger_free()
        self.hand_free()

    def free_no_delay(self):
        """复位并重置所有位置到初始状态（包含手指和手掌）"""
        # 额外的复位操作确保完全回归初始位置, 无delay
        self.move_fingers([1, 2, 3, 4, 5], [0, 0, 0, 0, 0])
        self.move_palms([1, 2, 3], [247, 450, 500], [1000, 1000, 1000])


    def demo(self):
        """执行预定义的完整演示序列"""
        self.boxing()
        time.sleep(1)
        self.one()
        time.sleep(1)
        self.ye()
        time.sleep(1.5)
        self.rock()
        time.sleep(1.5)
        self.index2thumb()
        time.sleep(1)
        self.middle2thumb()
        time.sleep(1)
        self.ring2thumb()
        time.sleep(1.5)
        self.back()
        time.sleep(1.5)
        self.dex_boxing()

    def start(self):
        """初始复位并开始演示"""
        self.free()
        time.sleep(2)
        for _ in range(200):
            self.demo()
            time.sleep(2.5)  # 演示循环间增加短暂停顿


# 使用示例
if __name__ == "__main__":
    hand_ctrl = DexHandControl()
    hand_ctrl.start()
