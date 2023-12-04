import bluetooth

bd_addr = '28:D0:EA:60:BC:E6'

port = 1

sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM )
sock.connect((bd_addr, port))

sock.send("hello!!")

sock.close()
