import sys
import time
import socket
import threading
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class Client(Node):
    def __init__(self):
        super().__init__("nano_17_proxy")
        self.s = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
        print("Socket initialized")
        self.port = 9999
        self.publisher_ = self.create_publisher(Float64MultiArray, '~/ft', 10)
        pass

    def connect(self):
        self.s.bind(('192.168.1.5',10000))
        self.s.connect(('192.168.1.7',self.port))
        print("socket connected")

    def recv(self):
        i  = 0
        while True:
            try:
                data = []
                buffer = self.s.recv(1024).decode()
                if i>5 :
                    data = json.loads(buffer)
                    msg = Float64MultiArray()
                    msg.data = data
                    self.publisher_.publish(msg)
                i+=1
            except KeyboardInterrupt:
                print("Interruped, exit")
                self.s.close()
                sys.exit()

if __name__ == "__main__":
    rclpy.init()
    client = Client()
    client.connect()
    client.recv()
    pass
