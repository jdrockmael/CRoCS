import bluetooth
import threading

serverMACAddress = '8C:88:2B:25:A9:4B'

port = 3
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

lock = threading.Lock()
flag = False

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

rec_thread = threading.Thread(target=rec_msg, args=(s,))
send_thread = threading.Thread(target=send_msg, args=(s,))

rec_thread.start()
send_thread.start()

rec_thread.join()
send_thread.join()

print("Closing sockect")

s.close()