import bluetooth
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

def handle_msg(croc, curr_client):
    while True:
        data = curr_client.recv(1024).split(b' ')
        if data:
            print(data)

            if data == b'quit':
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

socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
socket.bind(("", 4))

socket.listen(1)
print("Ready to connect")

for _ in range(2):
    client, info = socket.accept()
    print("Connecting to: ", croc_info(info[0]).name)
    list_of_clients[croc_info(info[0]).name] = client

for croc, curr_client in list_of_clients.items():
    curr_thread = threading.Thread(target=handle_msg, args=(croc, curr_client))
    client_threads.append(curr_thread)
    curr_thread.start()

for t in client_threads:
    t.join()

socket.close()