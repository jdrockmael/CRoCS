import bluetooth
from enum import Enum
from threading import Thread

class croc_info(Enum):
    croc00 = 0
    croc01 = 'B8:27:EB:71:7B:FB'
    croc02 = 'B8:27:EB:72:37:1B'
    croc03 = 3
    croc04 = 4
    croc05 = 5
    croc06 = 6

socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
socket.bind(("", 4))

socket.listen(1)
print("Ready to connect")

list_of_clients = {}

for _ in range(2):
    client, info = socket.accept()
    print("Connecting to: ", croc_info(info[0]).name)
    list_of_clients[croc_info(info[0]).name] = client

copy_of_clients = list_of_clients.copy()
while 1:
    list_of_clients = copy_of_clients.copy()
    if len(list_of_clients) == 0:
        socket.close()
        break

    for croc, curr_client in list_of_clients.items():
        data = curr_client.recv(1024).split(b' ')
        if data:
            print(data)

            try:
                list_of_clients[data[0].decode('ascii')].send(data[1])
            except:
                print(data[0], " is not a connected croc\n")

            if data == b'quit':
                copy_of_clients.pop(croc)
                print("Disconnected from ", croc)
                curr_client.close()
