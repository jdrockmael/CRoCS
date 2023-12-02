import bluetooth

sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )

port = 3
bd_addr = 'B8:27:EB:72:37:1B'

sock.bind(("",port))

sock.listen(1)
client_sock,address = sock.accept()

data = client_sock.recv(1024)
print("received [%s]" % data, " from ", address)

sock.connect((bd_addr, port))
sock.send("hello!!")

sock.close()