import bluetooth

port = 3
backlog = 1
size = 1024
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.bind(("", port))
s.listen(backlog)

client, clientInfo = s.accept()
while 1:
    data = client.recv(size)
    if data:
        print(data)
        if data == b'quit':
            break 
        text = input()
        client.send(text) # Echo back to client
        if text == "quit":
            break
        
print("Closing socket")
client.close()
s.close()