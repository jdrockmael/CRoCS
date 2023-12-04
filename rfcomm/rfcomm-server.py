import bluetooth

server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

port = 3
server_sock.bind(("",port))
server_sock.listen(1)

client_sock1,address1 = server_sock.accept()
print("Accepted connection from ",address1)

client_sock2,address2 = server_sock.accept()
print("Accepted connection from ",address2)

data1 = client_sock1.recv(1024)
print("received [%s]" % data1, " from ", address1)

data2 = client_sock2.recv(1024)
print("received [%s]" % data2, " from ", address2)

client_sock1.close()
client_sock2.close()
server_sock.close()
