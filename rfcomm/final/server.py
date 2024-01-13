import bluetooth
import os
from enum import Enum
import threading

class croc_info(Enum):
    croc00 = 0
    croc01 = 'B8:27:EB:71:7B:FB'
    croc02 = 'B8:27:EB:72:37:1B'
    croc03 = 3
    croc04 = 4
    croc05 = 5
    croc06 = 6

lock = threading.Lock()
list_of_clients = {}
client_threads = []

def num_of_crocs():
    croc_macs = [adr.value for adr in croc_info]
    nearby_devices = bluetooth.discover_devices()
    cnt = 0

    for curr_adr in nearby_devices:
        if curr_adr in croc_macs:
            cnt = cnt + 1
    
    return cnt


def handle_msg(croc, curr_client):
    while True:
        data = curr_client.recv(1024).split(b' ')
        if data:
            print(data)

            if data[0] == b'quit':
                with lock:
                    list_of_clients.pop(croc)
                print("Disconnected from ", croc)
                curr_client.close()
                break
            
            with lock:
                if data[0].decode('ascii') in list_of_clients:
                    list_of_clients[data[0].decode('ascii')].send(data[1])
                else:
                    print(data[0], " is not a connected croc\n")

os.system("bluetoothctl discoverable on")

socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
socket.bind(("", 5))

close_croc_cnt = num_of_crocs()
print(close_croc_cnt, "crocs detected")

socket.listen(1)
print("Ready to connect")

for _ in range(close_croc_cnt):
    client, info = socket.accept()
    print("Connecting to: ", croc_info(info[0]).name)
    list_of_clients[croc_info(info[0]).name] = client

print("All available crocs connected")

for croc, curr_client in list_of_clients.items():
    curr_thread = threading.Thread(target=handle_msg, args=(croc, curr_client))
    client_threads.append(curr_thread)
    curr_thread.start()

for t in client_threads:
    t.join()

print("closing server")
socket.close()
os.system("bluetoothctl discoverable off")