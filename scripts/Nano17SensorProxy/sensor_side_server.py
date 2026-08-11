import sys
import time
import socket
import threading
import json
import math
import nidaqmx
import nidaqmx.constants
import numpy as np

class Server:
    def __init__(self):
        print("Initializing Socket")
        self.s = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
        print("Socket initialized")
        self.port = 9999
        print("Binding socket")
        self.s.bind(('192.168.1.4',self.port))
        print(f"Socket bound to port {self.port}")
        print("Listening for connections now")
        self.s.listen(5) 

        self.task = nidaqmx.Task()
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai1")
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai2")
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai3")
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai4")
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai5")
        self.task.timing.cfg_samp_clk_timing(1000,samps_per_chan=50)

        self.cal_matrix = np.array([
            [ 0.006508136168,  0.004737349693, -0.039684981108, -1.643620133400,  0.018618559465,   1.690829515457],
            [-0.003218212631,  2.016800165176, -0.019988993183, -0.947898804539, -0.015476017259,  -0.999974071980],
            [ 1.851384401321,  0.053954269737,  1.880571126938,  0.067106232047,  1.906236290932,   0.054224062711],
            [-0.014561751857, 23.761793136597, 10.570952415466, -10.716448783875, -10.798959732056, -12.132943153381],
            [-11.938018798828, -0.492411583662,  6.568276405334, 19.510540008545,  6.146337985992, -19.568044662476],
            [-0.037593655288,  7.526563644409,  0.112936601043,  7.675779342651,  0.109928734601,   7.442779064178]
        ], dtype=np.float64)

        self.bias = np.dot(self.cal_matrix,np.array(self.task.read(nidaqmx.constants.READ_ALL_AVAILABLE))) # 6x50 matrix
        pass

    def keep_waiting_for_connection(self):
        while True:
            try:
                print("Waiting for client")
                # note that this is supposed to be blocking
                self.c,self.addr = self.s.accept()
                print(f"Got connection from {self.addr}")

                while True:
                        start_time = time.time()
                        data = self.read_ft_value() # double 6x50, the first list contains 50 values
                        for i in range(50):
                            ft_i = [data[0][i],data[1][i],data[2][i],data[3][i],data[4][i],data[5][i]]
                            ft_i.insert(0,start_time + 0.01 + i*0.001)
                            stream = json.dumps(ft_i)
                            stream += "\n"
                            self.c.send(stream.encode()) 
            except KeyboardInterrupt:

                print("Interrupted, exit")
                self.c.close()
                self.s.close()
                sys.exit()

    # replace function to read the ft data
    def read_ft_value(self):
        input_voltages = self.task.read(nidaqmx.constants.READ_ALL_AVAILABLE)
        signal_vector = np.array(input_voltages,dtype=float)
        ft_data=np.dot(self.cal_matrix,signal_vector) # 6x50
        ft_data -= self.bias
        return ft_data.tolist()

if __name__ == "__main__":
    server = Server()
    server.keep_waiting_for_connection()
    pass