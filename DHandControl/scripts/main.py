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

def set_rgb_color(ID, r, g, b):
    ip = query_ip_by_id(ID)
    message = {
        'Cmd': "RGB",
        'ID': ID,
        'R': r,
        'G': g,
        'B': b
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def position_up(ID):
    ip = query_ip_by_id(ID)
    message = {
        'Cmd': "Position+",
        'ID': ID
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def position_down(ID):
    ip = query_ip_by_id(ID)
    message = {
        'Cmd': "Position-",
        'ID': ID
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def write_pos_ex(ID, pos, vel, acc):
    ip = query_ip_by_id(ID)
    message = {
        'Cmd': "WritePosEx",
        'ID': ID,
        'pos': pos,
        'vel': vel,
        'acc': acc
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def reg_write_pos_ex(ID, pos, vel, acc):
    ip = query_ip_by_id(ID)
    message = {
        'Cmd': "RegWritePosEx",
        'ID': ID,
        'pos': pos,
        'vel': vel,
        'acc': acc
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def sync_write_pos_ex(ip, pos_list, vel_list, acc_list):
    message = {
        'Cmd': "SyncWritePosEx",
        'ID_list': query_id_list_by_ip(ip),
        'IDNum': len(query_id_list_by_ip(ip)),
        'pos_list': pos_list,
        'vel_list': vel_list,
        'acc_list': acc_list
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def get_state(ID):
    message = {
        'Cmd': "State",
        'ID': ID
    }
    ip = query_ip_by_id(ID)
    json_message = json.dumps(message)
    # send_udp_message(ip, UDP_PORT, json_message)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.4.10", 12345))
    sock.sendto(json_message.encode(), (ip, UDP_PORT))

    # print("Cmd sent")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            ip_address, port = addr
            # print(f"Received message: {data.decode()} from IP: {ip_address}, Port: {port}")
            return int(data.decode())
        except KeyboardInterrupt:
            print("Exiting...")
            break
    sock.close()

def enable_torque(ip, Enable):
    message = {
        'Cmd': "EnableTorque",
        'ID_list': query_id_list_by_ip(ip),
        'Enable': Enable
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def set_torque(ip, newTorque_list):

    message = {
        'Cmd': "SetTorque",
        'ID_list': query_id_list_by_ip(ip),
        'NewTorque_list': newTorque_list
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def set_mode(ip, Mode):
    message = {
        'Cmd': "SetMode",
        'ID_list': query_id_list_by_ip(ip),
        'Mode': Mode
    }
    json_message = json.dumps(message)
    send_udp_message(ip, UDP_PORT, json_message)

def set_time(ip, time_list, direction_list):
    message = {
        'Cmd': "SetTime",
        'ID_list': query_id_list_by_ip(ip),
        'Time_list': time_list,
        'Direction_list': direction_list
    }
    json_message = json.dumps(message)
    # send_udp_message(ip, UDP_PORT, json_message)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.4.10", 12345))
    sock.sendto(json_message.encode(), (ip, UDP_PORT))

    # print("Cmd sent")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            decoded_data = data.decode('utf-8')
            print(f"Received data: {decoded_data}")
            json_data = json.loads(decoded_data)
            pos_list = json_data["Pos_list"]
            speed_list = json_data["Speed_list"]
            current_list = json_data["Current_list"]

            return pos_list, speed_list, current_list
        except KeyboardInterrupt:
            print("Exiting...")
            break
    sock.close()








if __name__ == "__main__":
    # free()
    turn("192.168.4.5", 1)
























