#!/usr/bin/env python

# Created by Javier Leguina Peral
# 1 March 2020

import paramiko
import opti_ssr_package as opti
import math
from numpy import linalg as lalg


class ControlledSystem:
    """
    Definition of controlled system.
    Options to exert control input and get current state.
    """

    def __init__(self, IP_Address):
        # Set up SSH connection
        self.sshClient = paramiko.SSHClient()  # Initialise
        self.sshClient.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # Automatically set IP as "safe"
        self.sshClient.connect(hostname=IP_Address, username='root', password='')  # Establish connection
        self.stdin, self.stdout, self.stderr = self.sshClient.exec_command("./khepera4_test")  # Run test function
        print("SSH connection established successfully")

        # Initialise OptiTrack client
        self.optiClient = opti.OptiTrackClient()  # Set up remote Motive connection
        if self.optiClient:
            print("OptiTrack connection established successfully")
        else:
            print("ERROR: OptiTrack connection NOT established")

    def input(self, vL, vR):  # vL - velocity left; vR - velocity right
        # Khepera IV commands
        self.stdin.write("ms %(left)d %(right)d" % {"left": vL, "right":vR}) # Set control input

    def position(self):
        # Read and return current position
        pos, orient, time = self.optiClient.get_rigid_body(rb_id=0)  # Get position data
        pos = abs(pos)
        rotationX, rotationY, rotationZ = quaternion_to_euler(orient[1], orient[2], orient[3], orient[0])
        rotationZ = rotationZ + 180
        rotationZ = (rotationZ + 180) % (2 * 180) - 180
        return pos, rotationZ, time

    def deviation(self, target):
        x, foo, foo = self.position()
        d = lalg.norm(x - target)
        vec = [target[0] - x[0], target[1] - x[1]]
        alpha = 180 * math.atan(vec[1] / vec[0]) / math.pi
        if target[0] < x[0]:
            alpha = alpha - 180
            if target[1] > x[1]:
                alpha =alpha + 360
        return d, alpha

    def divergence(self, target):
        a, b = self.deviation(target)
        c, d, e = self.position()
        div = d - b
        return div

    def stop(self):
        # Stop controlled system motion
        self.stdin.write("s")

    def quit(self):
        # Quit motion control
        self.stdin.write("q")

    def close(self):
        # Close remote SSH connection
        self.sshClient.close()
        print("SSH connection closed successfully")


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def computeVelocities(d, phi):
    # alpha - angle between x & target
    # theta - current robot orientation angle
    # distance to objective
    v_max = 150
    if phi > 5:
        vL = v_max*abs(phi)/180
        vR = -v_max*abs(phi)/180
    elif phi < -5:
        vL = -v_max*abs(phi)/180
        vR = v_max*abs(phi)/180
    else:
        if abs(d) > 0.15:
            vL = v_max
            vR = v_max
        elif abs(d) <= 0.15:
            vL = v_max * d / 0.15
            vR = v_max * d / 0.15

    return vL, vR


def trajectoryControl(target):

    Distance, Angle = sys.deviation(target)

    while Distance > 0.0:
        pos, rotationZ, time = sys.position()
        div = sys.divergence(target)
        velocityLeft, velocityRight = computeVelocities(Distance, div)
        # print(div, Distance)
        sys.input(velocityLeft, velocityRight)
        Distance, Angle = sys.deviation(target)
        # print("Angle", Angle)
        # print("Orientation", rotationZ)
        if Distance < 0.02:
            for i in range(100):
                sys.input(0, 0)
            break



IP = "192.168.0.198"
position = [[0.1, 0.8, 0.07],
            [0.8, 0.1, 0.07],
            [0.8, 0.8, 0.07],
            [0.1, 0.1, 0.07],
            [0.1, 0.8, 0.07]]

sys = ControlledSystem(IP)

for positionFinal in position:
    trajectoryControl(positionFinal)

sys.quit()
sys.close()
print("198 Success")
