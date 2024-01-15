from enum import Enum
import bluetooth
import threading

class server_macs(Enum):
    jason = '28:D0:EA:60:BC:E6'
    tim = 'B0:DC:EF:86:98:0D'

lock = threading.Lock()
flag = False

def find_server_mac():
     possible_mac = [adr.value for adr in server_macs]
     nearby_devices = bluetooth.discover_devices()
     
     for curr_adr in nearby_devices:
          if curr_adr in possible_mac:
               print("connecting to ", server_macs(curr_adr).name)
               return curr_adr 
          
     return None

def rec_msg(socket):
     while True:
        with lock:
             if flag:
                  break
        print("waiting to recieve data")
        data = socket.recv(1024)
        if data:
            print(data)

def send_msg(socket):
    global flag
    while True:
            print("ready to take input")
            text = input()
            socket.send(text)

            if text == "quit":
                with lock:
                     flag = True
                break

s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((find_server_mac(), 4))

rec_thread = threading.Thread(target=rec_msg, args=(s,))
send_thread = threading.Thread(target=send_msg, args=(s,))

rec_thread.start()
send_thread.start()

rec_thread.join()
send_thread.join()

print("Closing sockect")

s.close()