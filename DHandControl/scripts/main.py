import json
import socket
import time
import cProfile
import timeit

# List of ESP32 IP addresses
ESP32_IPS = [
    "192.168.4.1", # AP
    "192.168.4.2", # Head
    "192.168.4.3", # Body
    "192.168.4.4"  # Tail
]

# PC IP address
PC_IP = "192.168.4.10"

ID_IP_table = {
    1: '192.168.4.2',
    2: '192.168.4.2',
    3: '192.168.4.2',
    4: '192.168.4.3',
    5: '192.168.4.3',
    6: '192.168.4.3',
    7: '192.168.4.4',
    8: '192.168.4.4',
    9: '192.168.4.4'
}

IP_IDList_table = {
    '192.168.4.2': [1, 2, 3],
    '192.168.4.3': [4, 5, 6],
    '192.168.4.4': [7, 8, 9]
}


UDP_PORT = 12345

def send_udp_message(ip, port, message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode(), (ip, port))
    sock.close()

def turn(ip,ID):
    message = {
        'Cmd': "Turn",
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def servo_move(ip, ID, position, time_in_ms):
    message = {
        'Cmd': "ServoMove",
        'ID': ID,
        'Pos': position,
        'Time': time_in_ms
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)



def query_ip_by_id(ID):
    return ID_IP_table.get(ID, "ID not found")

def query_id_list_by_ip(IP):
    return IP_IDList_table.get(IP, "IP not found")










if __name__ == "__main__":
    # free()
    # turn("192.168.4.5", 1)
    servo_move("192.168.4.5", 1, 500, 2000)
    # servo_move("192.168.4.5", 1, 500, 1000)
    # servo_move("192.168.4.5", 1, 1000, 1000)
    # servo_move("192.168.4.5", 1, 0, 1000)
    # servo_move("192.168.4.5", 1, 500, 1000)






















