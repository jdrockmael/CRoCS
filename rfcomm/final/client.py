import bluetooth

serverMACAddress = 'B8:27:EB:72:37:1B'

port = 3
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

while 1:
        text = input()
        s.send(text)
        if text == "quit":
            break
s.close()