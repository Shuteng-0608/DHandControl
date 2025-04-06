import json
import socket

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

def turn(ID):
    ip = "192.168.4.5"
    message = {
        'Cmd': "Turn",
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def servo_move(ID, position, time_in_ms):
    ip = "192.168.4.5"
    message = {
        'Cmd': "ServoMove",
        'ID': ID,
        'Pos': position,
        'Time': time_in_ms
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

def move_fingers(IDNum, ID_list, pos_list):
    ip = "192.168.4.5"
    message = {
        'Cmd': "MoveFingers",
        'IDNum': IDNum,
        'ID_list': ID_list,
        'pos_list': pos_list
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)




if __name__ == "__main__":
    # free()
    # turn(1)
    # servo_move(1, 500, 2000)
    # finger_move(1,500)
    move_fingers(5, [1,2,3,4,5], [100,200,300,400,500])























