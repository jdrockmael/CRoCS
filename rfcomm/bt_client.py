from enum import Enum
import bluetooth
import threading
from time import sleep

# list of bt mac addresses that will be running server
class server_macs(Enum):
    jason = '28:D0:EA:60:BC:E6'
    tim = 'B0:DC:EF:86:98:0D'

# lock so threads can share a variable 
lock = threading.Lock()

# shared variable so recieving thread can end when sending thread ends
# false if sending thread has not ended and true if it has
flag = False

# Finds which of the above listed macs are hosting the server and connect
# returns the sockect object used to send messages
# port is the port number that the server is bond to
def find_and_connect(port=4):
    possible_mac = [adr.value for adr in server_macs]
    nearby_devices = bluetooth.discover_devices()
    adr = None
     
    for curr_adr in nearby_devices:
        if curr_adr in possible_mac:
            print("connecting to", server_macs(curr_adr).name)
            adr = curr_adr

    if adr is not None:
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        for i in range(1, 4):
            try:
                sock.connect((adr, port))
                return sock
            except Exception as e:
                print("Failed to connect, try count:", i)
                print(e)
                sleep(1)
        print("Failed to connect to server")
        return None
    else:
        print("The server was not found")
        return None

# displays incoming messages
# takes the socket that has the server connected
def rec_msg(socket):
     while not flag:
        data = socket.recv(1024)
        if data:
            print(data)

# sends given messages over bluetooth
# takes the socket the server is connected to
def send_msg(socket):
    global flag
    while True:
            text = input()
            socket.send(text)

            if text == "quit":
                with lock:
                     flag = True
                break

if __name__ == '__main__':
    # make sockect
    s = find_and_connect(4)
    
    # create send and recieve thread
    rec_thread = threading.Thread(target=rec_msg, args=(s,))
    send_thread = threading.Thread(target=send_msg, args=(s,))

    # start threads
    rec_thread.start()
    send_thread.start()

    # kill threads once they end
    rec_thread.join()
    send_thread.join()

    # close server
    print("Closing socket")
    s.close()