import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import matplotlib.pyplot as plt
import numpy as np

import data
import experiment

SAMPLES_PER_STEP = 100*1.01
COUNTS_PER_UNIT = 1000000

plt.figure()
# for w in ['4','8','12']:
for w in ['32_10']:
    force_data = data.read('data\spring_{}.csv'.format(w),float=False,skip=7)
    tz = np.float_(force_data[' Tz'])/COUNTS_PER_UNIT
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

    # plt.figure()
    # plt.plot(tz)
    # plt.plot(tz_avg_i,tz_avg,'o')
    # plt.show()

    plt.plot(theta,tz_avg,label=w)

plt.xlabel('Angle[deg]')
plt.ylabel('Torque[Nm]')
plt.legend(title='Beam width[mm]')
plt.show()
