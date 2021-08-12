import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import time
import numpy as np
import matplotlib.pyplot as plt
from dynamixel_sdk import *

import data

DEG_2_RAD = np.pi/180

PORT_NAME = 'COM5'
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
SERVO_ID_0 = 1
SERVO_ID_1 = 2

SERVO_POS_0 = [664,590]
SERVO_POS_1 = [687,573]

portH = PortHandler(PORT_NAME)
portH.setBaudRate(BAUDRATE)
packetH = PacketHandler(PROTOCOL_VERSION)

p = 0.23
t_pre = time.time()
while True:
    # Sweeep to start position
    groupSW = GroupSyncWrite(portH, packetH, 30, 2)
    groupSW.addParam(SERVO_ID_0,SERVO_POS_0[0].to_bytes(2, 'little'))
    groupSW.addParam(SERVO_ID_1,SERVO_POS_1[0].to_bytes(2, 'little'))
    groupSW.txPacket()

    time.sleep(p/2)

    groupSW = GroupSyncWrite(portH, packetH, 30, 2)
    groupSW.addParam(SERVO_ID_0,SERVO_POS_0[1].to_bytes(2, 'little'))
    groupSW.addParam(SERVO_ID_1,SERVO_POS_1[1].to_bytes(2, 'little'))
    groupSW.txPacket()

    time.sleep(p/2)

    t = time.time()
    print(t-t_pre)
    t_pre = t

portH.closePort()
