import sys
import time
import socket
import threading
import json
import math

class Server:
    def __init__(self):
        print("Initializing Socket")
        self.s = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
        print("Socket initialized")
        self.port = 9999
        print("Binding socket")
        self.s.bind(('localhost',self.port))
        print(f"Socket bound to port {self.port}")
        print("Listening for connections now")
        self.s.listen(5)
        pass

    def keep_waiting_for_connection(self):
        while True:
            try:
                print("Waiting for client")
                # note that this is supposed to be blocking
                self.c,self.addr = self.s.accept()
                print(f"Got connection from {self.addr}")

                while True:
                        time.sleep(0.01)
                        try:
                            data = [self.read_ft_value(),time.time()]
                            stream = json.dumps(data)
                            stream += "\n"
                            self.c.send(stream.encode())
                        except BrokenPipeError:
                            print("Client Socket appears closed")
                            print(f"Client socket fd : {self.c.fileno()}")
                            break

            except KeyboardInterrupt:
                print("Interrupted, exit")
                self.c.close()
                self.s.close()
                sys.exit()

    # replace function to read the ft data
    def read_ft_value(self)->float:
        return 3.0*math.sin(time.time())

if __name__ == "__main__":
    server = Server()
    server.keep_waiting_for_connection()
    pass