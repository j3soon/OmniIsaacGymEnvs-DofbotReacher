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
