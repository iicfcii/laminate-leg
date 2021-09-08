import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import matplotlib.pyplot as plt
import numpy as np

import data
import experiment

SAMPLES_PER_STEP = 7000
COUNTS_PER_UNIT = 1000000

force_data = data.read('data\spring_300.csv')
tz = np.array(force_data['Tz'])/COUNTS_PER_UNIT
tz_init_i = data.detect_change(tz)
tz_init_i -= SAMPLES_PER_STEP/2

tz_avg_i = []
theta = []
tz_avg = []
for i, p in enumerate(experiment.POSITIONS):
    center_i = int(tz_init_i+i*SAMPLES_PER_STEP)
    start_i = int(center_i-SAMPLES_PER_STEP/4)
    end_i = int(center_i+SAMPLES_PER_STEP/4)
    tz_avg.append(np.average(tz[start_i:end_i]))

    tz_avg_i.append(center_i)
    theta.append((p-experiment.POSITION_START)*experiment.DEG_PER_COUNT*experiment.DEG_2_RAD)

plt.plot(theta,tz_avg)
plt.xlabel('Angle[deg]')
plt.ylabel('Torque[Nm]')
plt.show()
