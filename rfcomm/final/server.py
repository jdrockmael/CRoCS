import bluetooth
from enum import Enum
from threading import Thread

# setting up the socket
# NOTE: don't want to bother with passing by ref so made it a global
socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
socket.bind(("", 3))

class croc_info(Enum):
    # sets the croc information that comes from the accepts and gets its corresponding number
    # NOTE: replace the filler with the actual info that comes in for each croc
    # NOTE: if more crocs are added, they need to be registered in this enum
    fillercroc0_info = 0
    fillercroc1_info = 1
    fillercroc2_info = 2
    fillercroc3_info = 3
    fillercroc4_info = 4
    fillercroc5_info = 5
    fillercroc6_info = 6

def get_client_list(num_of_clients):
    list_of_clients = [None] * num_of_clients

    # NOTE: this process can be theaded to improve performance especially with more crocs
    for _ in range(num_of_clients):
        # accept connect from device attempting to connect
        # NOTE: I don't remember what clientInfo is we may be able to use this to id the specific croc
        backlog = 1
        # NOTE: can you call listen multiple times?
        # NOTE: may need to put this somewhere else if we go with threaded
        socket.listen(backlog)
        client, info = socket.accept()
        # NOTE: the info must be in a 'info' format but I don't know if it already is that format or if it needs to be converted
        # NOTE: have a check to make sure that the create_client didn't time out
        # NOTE: have a check so if the croc isn't defined in the enumerator an error is thrown
        croc_num = croc_info[info].value
        list_of_clients[croc_num] = client

    return list_of_clients

def read_clients():
    # make sure that this function returnes the reference to the client objects and not a copy
    client_list = get_client_list(6)
    size = 1024
    while 1:
        for i, curr_client in enumerate(client_list):
            # if client is not none
            if curr_client:
                data = curr_client.recv(size)
                if data:
                    print(data)
                    if data == b'quit':
                        client_list[i] = None
                        curr_client.close()

            if not any(curr_client):
                socket.close()
                return 

print("Starting to read client")
read_clients()   
print("Closing socket")