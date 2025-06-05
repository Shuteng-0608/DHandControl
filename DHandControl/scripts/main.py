import json
import socket
import time

# List of ESP32 IP addresses
ESP32_IPS = [
    "192.168.4.1", # AP
    "192.168.4.5"  # Hand
]

# PC IP address
PC_IP = "192.168.4.10"


UDP_PORT = 12345

def send_udp_message(ip, port, message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode(), (ip, port))
    sock.close()

# def turn(ID):
#     ip = "192.168.4.5"
#     message = {
#         'Cmd': "Turn",
#     }
#     json_message = json.dumps(message)
#     send_udp_message(ip, UDP_PORT, json_message)

def servo_move(ID, position, time_in_ms):
    """
    Init Pos:
        - [1] : *247* - 1000
        - [2] : 649 - *500* - 131
        - [3] : 426 - *500* - 575
    :param ID:
    :param position:
    :param time_in_ms:
    :return:
    """
    ip = "192.168.4.5"
    message = {
        'Cmd': "ServoMove",
        'ID': ID,
        'Pos': position,
        'Time': time_in_ms
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def move_palms(ID_list, pos_list, time_list):
    ip = "192.168.4.5"
    message = {
        'Cmd': "MovePalms",
        'ID_list': ID_list,
        'pos_list': pos_list,
        'time_list': time_list
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)


def finger_move(ID, position):
    ip = "192.168.4.5"
    message = {
        'Cmd': "FingerMove",
        'ID': ID,
        'Pos': position
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def move_fingers(ID_list, pos_list):
    ip = "192.168.4.5"
    message = {
        'Cmd': "MoveFingers",
        'ID_list': ID_list,
        'pos_list': pos_list
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def clear_error(ID):
    ip = "192.168.4.5"
    message = {
        'Cmd': "ClearError",
        'ID': ID,
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def boxing():
    move_fingers([2, 3, 4, 5], [2000, 2000, 2000, 2000])
    time.sleep(0.4)
    move_fingers([1], [690])
    time.sleep(3)
    free()

def index2thumb():
    move_fingers([1, 2], [600, 1330])
    time.sleep(3)
    free()

def middle2thumb():
    move_fingers([1, 3], [1130, 1700])
    time.sleep(3)
    free()

def ring2thumb():
    # servo_move(2, 380, 500)
    # servo_move(1, 530, 500)
    move_palms([1], [300], [1000])
    move_palms([2, 1], [380, 530], [1000, 1000])
    # time.sleep(3)
    move_fingers([1, 4], [820, 1360])
    time.sleep(3)
    free()

def finger_free():
    """
        ==== ATTENTION ==== : 执行之前需要确认不会发生干涉
    """
    move_fingers([1, 2, 3, 4, 5], [0, 0, 0, 0, 0])
    time.sleep(1)

def hand_free():
    """
        ==== ATTENTION ==== : 执行之前需要确认不会发生干涉
    """
    # servo_move(1, 247, 500)
    # servo_move(2, 500, 500)
    # servo_move(3, 500, 500)
    move_palms([1, 2, 3], [247, 500, 500], [1000, 1000, 1000])

def hex_boxing():
    move_palms([1, 2], [1000, 131], [1000, 1000])
    # time.sleep(3)
    move_fingers([2, 3, 5], [2000, 2000, 2000])
    time.sleep(0.8)
    move_fingers([1, 4], [210, 1060])
    time.sleep(3)
    free()

def free():
    """
    ==== ATTENTION ==== : 执行之前需要确认不会发生干涉
    """
    finger_free()
    hand_free()


if __name__ == "__main__":
    # clear_error(1)
    # clear_error(2)
    # clear_error(3)
    # clear_error(4)
    # clear_error(5)
    # ================= #
    free()
    time.sleep(2)
    for i in range(100):
        boxing()
        time.sleep(2)
        index2thumb()
        time.sleep(2)
        middle2thumb()
        time.sleep(2)
        ring2thumb()
        time.sleep(2)
        hex_boxing()
        time.sleep(2)


    # ================= #
    # servo_move(2, 380, 1000)
    # servo_move(1, 530, 1000)
    # free()
    # move_palms([1, 2], [530, 380], [1000, 1000])




    # free() #
    # turn(1)
    # servo_move(1, 0, 2000)
    # time.sleep(2)
    # finger_move(5,500)
    # finger_move(2,1000)
    # finger_move(3, 1000)
    # finger_move(4, 1000)
    # finger_move(5, 1000)
    # finger_move(2, 500)
    # finger_move(3, 500)
    # finger_move(4, 500)
    # finger_move(5, 500)
    # finger_move(4, 500)
    # servo_move(1, 426, 100)

    # move_fingers([2,3,4,5], [500,500,500,500])
    # time.sleep(2)
    # move_fingers([2,3,4,5], [1000,1000,1000,1000])
    # time.sleep(2)
    # move_fingers([2,3,4,5], [750,750,750,750])
    # move_fingers([2],[0])
    # move_fingers([4], [0])
    pass






















