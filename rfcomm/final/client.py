import bluetooth

serverMACAddress = '28:D0:EA:60:BC:E6'

port = 4
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

while 1:
        text = input()
        s.send(text)
        if text == "quit":
            break
s.close()