#!/usr/bin/env python
# Echo client program

import socket
import sys

HOST = "192.168.0.100"  # The UR IP address
PORT = 30002  # UR secondary client
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

path = '/home/davide/catkin_ws/src/soft_robotics/soft_robotics_description/scripts/'


def callback(data):

    script = ''
    if(data == 'close'):
        script = path + 'close.script'
    elif(data == 'open'):
        script = path + 'open.script'
    else:
        print("Invalid argument!")
    f = open(script, "rb")  # Robotiq Gripper
    l = f.read(2024)
    while (l):
        s.send(l)
        print(l)
        l = f.read(2024)
    f.close()
    print(str(data))

    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    callback('open')
