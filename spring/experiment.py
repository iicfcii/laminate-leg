import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import time
import numpy as np
import matplotlib.pyplot as plt
from dynamixel_sdk import *

DEG_2_RAD = np.pi/180

DEG_PER_COUNT = 0.088
POSITION_START = 2560
POSITION_RANGE = 300 # 0.088 deg/pulse
POSITION_STEP = 10
assert POSITION_RANGE%POSITION_STEP == 0
POSITIONS = np.concatenate((
    np.arange(POSITION_START,POSITION_START+POSITION_RANGE/2,POSITION_STEP),
    np.arange(POSITION_START+POSITION_RANGE/2,POSITION_START-POSITION_RANGE/2,-POSITION_STEP),
    np.arange(POSITION_START-POSITION_RANGE/2,POSITION_START,POSITION_STEP),
))
# POSITIONS = np.tile(POSITIONS,(5))
POSITIONS = np.concatenate((
    POSITIONS,
    [POSITION_START],
))

ADDR_TORQUE_ENABLE = 64
LEN_TORQUE_ENABLE = 1
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4

PORT_NAME = 'COM9'
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
SERVO_ID = 1

if __name__ == '__main__':
    portH = PortHandler(PORT_NAME)
    portH.setBaudRate(BAUDRATE)
    packetH = PacketHandler(PROTOCOL_VERSION)

    # Disable torque and check current position
    # To avoid potential damage to test setup, especially force sensor
    groupSW = GroupSyncWrite(portH, packetH, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE)
    groupSW.addParam(SERVO_ID,int(0).to_bytes(LEN_TORQUE_ENABLE, 'little'))
    groupSW.txPacket()

    groupSR = GroupSyncRead(portH, packetH, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    groupSR.addParam(SERVO_ID)
    groupSR.txRxPacket()
    current_pos = groupSR.getData(SERVO_ID,ADDR_PRESENT_POSITION,LEN_PRESENT_POSITION)


    p_start = POSITION_START-POSITION_RANGE/2
    p_end = POSITION_START+POSITION_RANGE/2
    assert current_pos > p_start and current_pos < p_end, 'Position should be between {} and {} but is {}'.format(p_start,p_end,current_pos)

    # Enable torque and move to start position
    groupSW = GroupSyncWrite(portH, packetH, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE)
    groupSW.addParam(SERVO_ID,int(1).to_bytes(LEN_TORQUE_ENABLE, 'little'))
    groupSW.txPacket()

    groupSW = GroupSyncWrite(portH, packetH, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    groupSW.addParam(SERVO_ID,int(POSITION_START).to_bytes(LEN_GOAL_POSITION, 'little'))
    groupSW.txPacket()

    print('Homed')
    time.sleep(3)

    for p in POSITIONS:
        groupSW = GroupSyncWrite(portH, packetH, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        groupSW.addParam(SERVO_ID,int(p).to_bytes(LEN_GOAL_POSITION, 'little'))
        groupSW.txPacket()
        print('Current position',p)
        time.sleep(1)

    # Disable torque
    groupSW = GroupSyncWrite(portH, packetH, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE)
    groupSW.addParam(SERVO_ID,int(0).to_bytes(LEN_TORQUE_ENABLE, 'little'))
    groupSW.txPacket()
