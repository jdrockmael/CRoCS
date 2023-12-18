import bluetooth
import threading

serverMACAddress = '8C:88:2B:25:A9:4B'

port = 4
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

event = False

def rec_msg(socket):
     while not event:
        print("waiting to recieve data")
        data = socket.recv(1024)
        if data:
            print(data)

def send_msg(socket):
    while True:
            print("ready to take input")
            text = input()
            socket.send(text)

            if text == "quit":
                event = True
                break

rec_thread = threading.Thread(target=rec_msg, args=(s))
send_thread = threading.Thread(target=send_msg, args=(s))

rec_thread.start()
send_thread.start()

rec_thread.join()
send_thread.join()

print("Closing sockect")

s.close()