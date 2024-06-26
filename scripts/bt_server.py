import bluetooth
import os
from enum import Enum
import threading
import rospy

# enum of all the crocs bt mac adress
class croc_info(Enum):
    croc00 = 'B8:27:EB:E3:F8:83'
    croc01 = 'B8:27:EB:71:7B:FB'
    croc02 = 'B8:27:EB:72:37:1B'
    croc03 = 'B8:27:EB:CA:AA:70'
    croc04 = 'B8:27:EB:CB:AB:FD'
    croc05 = 'B8:27:EB:DC:84:5B'
    croc06 = 'B8:27:EB:10:4E:19'

# lock so threads can used shared variables
lock = threading.Lock()

# dictionary relating croc names to client objects used to send messages
list_of_clients = {}

#dictionary of croc positions
pos_list = {}

# the number of crocs that are connected
connect_croc_cnt = None

# sets up and connects crocs to server
# port is the port that you want the server to be bound to
def setup(port=4):
    global connect_croc_cnt
    croc_macs = [adr.value for adr in croc_info]
    nearby_devices = bluetooth.discover_devices()
    cnt = 0

    for curr_adr in nearby_devices:
        if curr_adr in croc_macs:
            cnt = cnt + 1

    # makes the server discoverable
    os.system("bluetoothctl discoverable on")

    socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    socket.bind(('', port))

    connect_croc_cnt = cnt
    print(connect_croc_cnt, "crocs detected")

    socket.listen(1)
    print("Ready to connect")

    # connects to crocs and stores their client object in global dicitonary
    for _ in range(connect_croc_cnt):
        client, info = socket.accept()
        print("Connecting to:", croc_info(info[0]).name)
        list_of_clients[croc_info(info[0]).name] = client

    print("All available crocs connected")
    return socket

# takes messages and sends them to correct croc
# croc is the croc sending the message
# data is an array of b'' with 0 being the croc target
# and data[1] is the message being sent
def handle_msg(croc, data):
    global pos_list
    name = data[0].decode('ascii')
    if name == 'server':
        print(data[1], "from", croc)
        if b'locked' not in data[1]:
            pos_list[croc] = data[1]
        elif b'locked' in data[1]:
            list_of_clients[croc].send(b'you_are_locked')
    elif name in list_of_clients:
        if b'path' not in data[1]:
            pos_list[croc] = data[1]
        elif b'path' in data[1]:
            list_of_clients[croc].send(b'path:' + pos_list[name])
        list_of_clients[name].send(data[1])
    else:
        print(data[0], "is not a connected croc\n")

# takes messages from bt and uses handle message
# croc is current croc recieveing the message
# curr_client is the related client object with the current croc
def rec_msg(croc, curr_client):
    global connect_croc_cnt
    while True:
        data = curr_client.recv(1024).split(b' ')
        if data:
            print(data)

            if data[0] == b'quit':
                with lock:
                    connect_croc_cnt -= 1
                    list_of_clients.pop(croc)
                print("Disconnected from", croc)
                curr_client.close()
                break
            
            handle_msg(croc=croc, data=data)

# sends messages by taking in inputs and using handle message function
def send_msg():
    while connect_croc_cnt > 0:
        text = input().encode('ascii').split(b' ')
        handle_msg('server', text)

if __name__ == "__main__":
    s = setup(4)

    # starts the thread used for sending messages
    send_thread = threading.Thread(target=send_msg, args=())
    send_thread.start()

    # starts a thread for each client
    client_threads = []
    for croc, curr_client in list_of_clients.items():
        curr_thread = threading.Thread(target=rec_msg, args=(croc, curr_client))
        client_threads.append(curr_thread)
        curr_thread.start()

    # paths = [None] * list_of_clients.size()
    time_step = 0
    paths = {}
    if rospy.has_param("/cbs_output"):
            for i in range(len(paths)):     # Replace this for loop to iterating through list of clients instead 
            
                paths[i] = rospy.get_param("/cbs_output/schedule/croc" + str(i+1))      

    longest_path = max(paths, key= lambda croc: len(set(paths[croc])))


    step = paths[time_step]
    x, y = step['x'], step['y']

    # kill threads once they finish
    send_thread.join()
    for t in client_threads:
        t.join()

    # closes server and disables discoverable
    print("closing server")
    s.close()
    os.system("bluetoothctl discoverable off")
