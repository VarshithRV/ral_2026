import socket
import time

UR_IP = "192.168.1.102"
UR_PORT = 30003

# Define two target poses: [x, y, z, rx, ry, rz] in meters/radians
point1 = [0.400, 0.000, 0.300, 0.0, 3.1416, 0.0]
point2 = [0.450, 0.050, 0.300, 0.0, 3.1416, 0.0]

# Speed, acceleration, and blend radius
a = 0.5      # acceleration (m/s^2)
v = 0.05     # velocity (m/s)
r = 0.001    # blend radius (m) - small blend for smooth transition, 0 for exact stop

def build_movep(pose, a, v, r):
    p = "p[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}]".format(*pose)
    return "movep({}, a={}, v={}, r={})\n".format(p, a, v, r)

script = "def move_prog():\n"
script += "  " + build_movep(point1, a, v, 0.0)   # r=0 on last/first point to avoid blend overshoot
script += "  " + build_movep(point2, a, v, 0.0)
script += "end\n"

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((UR_IP, UR_PORT))
sock.send(script.encode('utf-8'))

time.sleep(2)  # replace with your idle-detection / RT speed polling for real use
sock.close