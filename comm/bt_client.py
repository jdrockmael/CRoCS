import rospy
from enum import Enum
import bluetooth
from std_msgs.msg import String
from time import sleep

# list of bt mac addresses that will be running server
class server_macs(Enum):
    jason = '28:D0:EA:60:BC:E6'
    tim = 'B0:DC:EF:86:98:0D'

# publisher that publishes incoming messages from the server
in_msgs_pub = rospy.Publisher('incoming_msgs', String, queue_size=10)

# Finds which of the above listed macs are hosting the server and connect
# returns the sockect object used to send messages
# port is the port number that the server is bond to
def find_and_connect(port=4):
    possible_mac = [adr.value for adr in server_macs]
    nearby_devices = bluetooth.discover_devices()
     
    for curr_adr in nearby_devices:
        if curr_adr in possible_mac:
            print("connecting to", server_macs(curr_adr).name)
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            for i in range(1, 4):
                try:
                    sock.connect((curr_adr, port))
                    if sock is not None:
                        return sock
                    else:
                        raise Exception("socket was none")
                except Exception as e:
                    print("Failed to connect, try count:", i)
                    print(e)
                    sleep(1)
        
    print("The server was not found or could not connect")
    return None

# make sockect for gloabal use
s = find_and_connect(port=4)

# sends given messages over bluetooth
# takes the socket the server is connected to
def send_msg(data):
    print(data)
    s.send(str(data))

if __name__ == '__main__':
    # init node
    rospy.init_node('client_comm')

    # subscribing to the out_going msgs topic that is sent to server
    rospy.Subscriber("out_going_msgs", String, send_msg)
    
    # pubs the incoming messages from server
    while not rospy.is_shutdown():
        data = s.recv(1024)
        if data:
            in_msgs_pub.publish(data.decode())

    # close server
    print("Closing socket")
    s.close()