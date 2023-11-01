# Copyright (c) 2022-2023, Johnson Sun
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import socket
import struct
import time

import numpy as np


class RealWorldDofbot():
    # Defined in dofbot.usd
    sim_dof_angle_limits = [
        (-90, 90, False),
        (-90, 90, False),
        (-90, 90, False),
        (-90, 90, False),
        (-90, 180, False),
        (-30, 60, True),
        # (-30, 60): /arm_01/link5/Finger_Left_01/Finger_Left_01_RevoluteJoint
        # (-60, 30): /arm_01/link5/Finger_Right_01/Finger_Right_01_RevoluteJoint
    ] # _sim_dof_limits[:,2] == True indicates inversed joint angle compared to real

    # Ref: Section `6.5 Control all servo` in  http://www.yahboom.net/study/Dofbot-Jetson_nano
    servo_angle_limits = [
        (0, 180),
        (0, 180),
        (0, 180),
        (0, 180),
        (0, 270),
        (0, 180),
    ]
    def __init__(self, IP, PORT, fail_quietely=False, verbose=False) -> None:
        print("Connecting to real-world Dofbot at IP:", IP, "and port:", PORT)
        self.fail_quietely = fail_quietely
        self.failed = False
        self.last_sync_time = 0
        self.sync_hz = 10000
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address = (IP, PORT)
            self.sock.connect(server_address)
            print("Connected to real-world Dofbot!")
        except socket.error as e:
            self.failed = True
            print("Connection to real-world Dofbot failed!")
            if self.fail_quietely:
                print(e)
            else:
                raise e

    def send_joint_pos(self, joint_pos):
        if time.time() - self.last_sync_time < 1 / self.sync_hz:
            return
        self.last_sync_time = time.time()
        if len(joint_pos) != 6:
            raise Exception("The length of Dofbot joint_pos is {}, but should be 6!".format(len(joint_pos)))
        # Convert Sim angles to Real angles
        servo_angles = [90] * 6
        for i, pos in enumerate(joint_pos):
            if i == 5:
                # Ignore the gripper joints for Reacher task
                continue
            # Map [L, U] to [A, B]
            L, U, inversed = self.sim_dof_angle_limits[i]
            A, B = self.servo_angle_limits[i]
            angle = np.rad2deg(float(pos))
            if not L <= angle <= U:
                print("The {}-th simulation joint angle ({}) is out of range! Should be in [{}, {}]".format(i, angle, L, U))
                angle = np.clip(angle, L, U)
            servo_angles[i] = (angle - L) * ((B-A)/(U-L)) + A # Map [L, U] to [A, B]
            if inversed:
                servo_angles[i] = (B-A) - (servo_angles[i] - A) + A # Map [A, B] to [B, A]
            if not A <= servo_angles[i] <= B:
                raise Exception("(Should Not Happen) The {}-th real world joint angle ({}) is out of range! hould be in [{}, {}]".format(i, servo_angles[i], A, B))
        print("Sending real-world Dofbot joint angles:", servo_angles)
        if self.failed:
            print("Cannot send joint states. Not connected to real-world Dofbot!")
            return
        packer = struct.Struct("f f f f f f")
        packed_data = packer.pack(*servo_angles)
        try:
            self.sock.sendall(packed_data)
        except socket.error as e:
            self.failed = True
            print("Send to real-world Dofbot failed!")
            if self.fail_quietely:
                print(e)
            else:
                raise e

if __name__ == "__main__":
    IP = input("Enter Dofbot's IP: ")
    PORT = input("Enter Dofbot's Port: ")
    dofbot = RealWorldDofbot(IP, int(PORT))
    pos = [np.deg2rad(0)] * 6
    dofbot.send_joint_pos(pos)
    print("Dofbot joint angles reset.")
