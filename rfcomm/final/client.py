import bluetooth
import threading

serverMACAddress = '8C:88:2B:25:A9:4B'

port = 4
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

exit_event = threading.Event()

def rec_msg(socket, event):
     while not event.is_set():
        print("waiting to recieve data")
        data = socket.recv(1024)
        if data:
            print(data)

def send_msg(socket, event):
    while True:
            print("ready to take input")
            text = input()
            socket.send(text)

            if text == "quit":
                event.set()
                break

rec_thread = threading.Thread(target=rec_msg, args=(s, exit_event))
send_thread = threading.Thread(target=send_msg, args=(s, exit_event))

rec_thread.start()
send_thread.start()

rec_thread.join()
send_thread.join()

s.close()