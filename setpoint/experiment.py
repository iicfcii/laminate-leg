import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import time
import numpy as np
import matplotlib.pyplot as plt
from dynamixel_sdk import *

import data

DEG_2_RAD = np.pi/180

PORT_NAME = 'COM3'
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
SERVO_ID = 1

portH = PortHandler(PORT_NAME)
portH.setBaudRate(BAUDRATE)
packetH = PacketHandler(PROTOCOL_VERSION)

# Move to 0
groupSW = GroupSyncWrite(portH, packetH, 30, 2)
goal_pos = int(0/300*1023)
groupSW.addParam(SERVO_ID,goal_pos.to_bytes(2, 'little'))
groupSW.txPacket()

time.sleep(1.0)

# Move to desired angle
deg = 135
groupSW = GroupSyncWrite(portH, packetH, 30, 2)
goal_pos = int(deg/300*1023)
groupSW.addParam(SERVO_ID,goal_pos.to_bytes(2, 'little'))
groupSW.txPacket()

# Collect data
groupSR = GroupSyncRead(portH, packetH, 37, 2)
groupSR.addParam(SERVO_ID)

init_t = time.time()

exp_data = {
    't': [],
    'pos': []
}
while len(exp_data['t']) == 0 or exp_data['t'][-1] < 1:
    groupSR.txRxPacket()
    current_t = time.time()-init_t
    current_pos = groupSR.getData(SERVO_ID,37,2)/1023*300/180*np.pi

    if current_pos == 0: continue # data loss

    exp_data['t'].append(current_t)
    exp_data['pos'].append(current_pos)
    print(exp_data['t'][-1],exp_data['pos'][-1])

portH.closePort()

# Save data
file_name = 'data/setpoint_{:d}.csv'.format(deg)
data.write(
    file_name,
    list(exp_data.keys()),
    list(exp_data.values())
)

plt.figure()
plt.plot(exp_data['t'],exp_data['pos'])
plt.show()
