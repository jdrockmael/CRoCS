import bluetooth

hostMACAddress = 'B8:27:EB:72:37:1B' # The MAC address of a Bluetooth adapter on the server. The server might have multiple Bluetooth adapters.
port = 3
backlog = 1
size = 1024
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.bind((hostMACAddress, port))
s.listen(backlog)

client, clientInfo = s.accept()
while 1:
    data = client.recv(size)
    if data:
        print(data)
        if data == b'quit':
            break 
        text = input()
        s.send(text) # Echo back to client
        if text == "quit":
            break
        
print("Closing socket")
client.close()
s.close()