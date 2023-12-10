import bluetooth
import threading

serverMACAddress = '28:D0:EA:60:BC:E6'

port = 4
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

exit_event = threading.Event()

def rec_msg(socket, event):
     while not event.is_set():
        data = socket.recv(1024)
        if data:
            print(data)

def send_msg(socket, event):
    while True:
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