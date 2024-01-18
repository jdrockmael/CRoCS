import bluetooth
import os
from enum import Enum
import threading

class croc_info(Enum):
    croc00 = 0
    croc01 = 'B8:27:EB:71:7B:FB'
    croc02 = 'B8:27:EB:72:37:1B'
    croc03 = 'B8:27:EB:CA:AA:70'
    croc04 = 'B8:27:EB:CB:AB:FD'
    croc05 = 5
    croc06 = 'B8:27:EB:10:4E:19'

lock = threading.Lock()
list_of_clients = {}
client_threads = []
close_croc_cnt = None

def num_of_crocs():
    croc_macs = [adr.value for adr in croc_info]
    nearby_devices = bluetooth.discover_devices()
    cnt = 0

    for curr_adr in nearby_devices:
        if curr_adr in croc_macs:
            cnt = cnt + 1
    
    return cnt

def handle_msg(croc, data):
    name = data[0].decode('ascii')
    if name == 'server':
        print(data[1], "from", croc)
    elif name in list_of_clients:
        list_of_clients[name].send(data[1])
    else:
        print(data[0], "is not a connected croc\n")

def rec_msg(croc, curr_client):
    global close_croc_cnt
    while True:
        data = curr_client.recv(1024).split(b' ')
        if data:
            print(data)

            if data[0] == b'quit':
                with lock:
                    close_croc_cnt -= 1
                    list_of_clients.pop(croc)
                print("Disconnected from", croc)
                curr_client.close()
                break
            
            with lock:
                handle_msg(croc=croc, data=data)

def send_msg(text):
    print("ready to take input")
    text = input().encode('ascii').split(b' ')

    with lock:
        handle_msg('server', text)

def run_test():
    for i in range(6):
        send_msg("server server_saying_hello_world")
        for j in ["1", "2", "3", "4", "6"]:
            send_msg("croc0" + j, "sending_message_hello_world")

print
os.system("bluetoothctl discoverable on")

socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
socket.bind(('', 4))

close_croc_cnt = num_of_crocs()
print(close_croc_cnt, "crocs detected")

socket.listen(1)
print("Ready to connect")

for _ in range(close_croc_cnt):
    client, info = socket.accept()
    print("Connecting to:", croc_info(info[0]).name)
    list_of_clients[croc_info(info[0]).name] = client

print("All available crocs connected")

send_thread = threading.Thread(target=run_test, args=())
send_thread.start()

for croc, curr_client in list_of_clients.items():
    curr_thread = threading.Thread(target=rec_msg, args=(croc, curr_client))
    client_threads.append(curr_thread)
    curr_thread.start()

send_thread.join()

for t in client_threads:
    t.join()

print("closing server")
socket.close()
os.system("bluetoothctl discoverable off")