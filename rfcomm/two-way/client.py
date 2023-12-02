import bluetooth

serverMACAddress = 'B8:27:EB:72:37:1B'
port = 3
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

text = input()
s.send(text)

s.bind((serverMACAddress, port))
s.listen(1)

client, clientInfo = s.accept()
while 1:
    data = client.recv(1024)
    if data:
        print(data)
        if data == b'quit':
            break 
        text = input()
        s.send(text)
        if text == "quit":
            break
s.close()