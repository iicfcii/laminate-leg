import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def FourBarCalc(bad: float, ad: float, ab: float, bc: float, cd: float):
    """
    Calculate the joint positions of a four-bar linkage

    Uses only the cosine rule to avoid multiple solutions

    :param bad: Angle between base and link 1 (rad)
    :param ad: Link 0 length (base)
    :param ab: Link 1 length
    :param bc: Link 2 length
    :param cd: Link 3 length
    :return: point coords, angles
    """
    bd = np.sqrt(ab**2+ad**2-2*ab*ad*np.cos(bad))
    abd = np.arccos((ab**2+bd**2-ad**2)/(2*ab*bd))
    adb = np.arccos((ad**2+bd**2-ab**2)/(2*ad*bd))
    cbd = np.arccos((bc**2+bd**2-cd**2)/(2*bc*bd))
    bcd = np.arccos((bc**2+cd**2-bd**2)/(2*bc*cd))
    bdc = np.arccos((bd**2+cd**2-bc**2)/(2*bd*cd))

    adc = adb + bdc
    abc = abd + cbd

    pts = np.array([
        [0,0],
        [ab*np.cos(bad),ab*np.sin(bad)],
        [cd*np.cos(np.pi-adc)+ad,cd*np.sin(np.pi-adc)],
        [ad,0]
    ])

    ts = np.array([bad, abc, bcd, adc])

    return pts, ts

DEG_2_RAD = np.pi/180

# Experiment parameters
DEG_PER_COUNT = -0.088 # 0.088 deg/pulse, angle decrease with count increase
POSITION_RANGE = 800   # +/- 400
POSITION_STEP = 10     # 10 counts per step
POSITION_LIMITS = [-25, 40]
print('Expected counter value: ' + str((2*POSITION_RANGE)/POSITION_STEP))

# Structure geometry
link_ab = 0.020  # m
link_bc = 0.010  # m
init_bad = 30    # deg

# Open torque data
df = pd.read_csv (r'tz.csv')
data = df.to_numpy()
print('Input table size: ' + str(data.shape))

last_slope_dir = 0
triggered = False
counters = np.zeros(data.shape[1])

for c in range(data.shape[1]):
    zero_value = np.average(data[:50, c])
    position = 0
    results = []

    # Initialize plot
    fig, axes = plt.subplots(2, 2)
    fig.suptitle(df.columns.values[c], fontsize=16)
    axes[0, 0].set_title('Linkage Trajectory')

    for r in range(data.shape[0]-3):
        data_point = data[r, c] - zero_value
        data_point_next = data[r+3, c] - zero_value
        if np.isnan(data_point_next):
            print('Col ' + str(c) + ' stopping at row ' + str(r))
            break

        slope = data_point_next - data_point

        if slope > 0:
            slope_dir = 1
        else:
            slope_dir = -1

        if slope_dir != last_slope_dir:
            last_slope_dir = slope_dir
            triggered = False

        if (abs(slope) > 0.00025) and not triggered:
            # Support moment
            Tz = data_point

            # Limited position
            pos = position
            if position < POSITION_LIMITS[0]:
                pos = POSITION_LIMITS[0]
            elif position > POSITION_LIMITS[1]:
                pos = POSITION_LIMITS[1]

            # 4-bar position
            theta0 = (init_bad + pos) * DEG_2_RAD
            points, thetas = FourBarCalc(theta0, link_ab/2, link_ab, link_bc, link_ab/2)

            # Plot every 10th position
            if counters[c]%10 == 0:
                xs = points[:, 0]
                ys = points[:, 1]
                axes[0, 0].plot(xs, ys)

            # Spring displacement
            phi = thetas[3] - np.pi

            # Force angle
            alpha = thetas[0] + thetas[1] - np.pi/2
            beta = np.pi - thetas[0] - thetas[1]
            gamma = thetas[3] - np.pi/2
            delta = np.pi - thetas[3]

            assert (alpha + beta + gamma + delta - np.pi) < 0.000001, 'Angles do not sum to 180'

            # Spring moment
            r = link_ab/2
            num = Tz/(r*np.sin(beta))
            den = 1/(r*np.sin(beta)) + 1/(r*np.cos(beta)*np.sin(delta) + r*np.sin(beta)*np.sin(gamma))
            M = num/den

            # Force
            F = (Tz - M)/(r*np.sin(beta))
            Fx = F * np.cos(beta)
            Fy = F * np.sin(beta)

            # Store results at this position
            results.append([pos, phi/DEG_2_RAD, phi, Tz, M, F, Fx, Fy])

            # Update system state
            counters[c] = counters[c] + 1
            position = position + slope_dir*POSITION_STEP*DEG_PER_COUNT
            triggered = True

    results = np.array(results)

    axes[0, 1].set_title('Spring Profile')
    axes[0, 1].plot(results[:, 2], results[:, 4])
    axes[0, 1].set_xlim([-1.4, 1.0])
    axes[0, 1].set_ylim([-0.06, 0.06])
    axes[0, 1].set_xticks(np.linspace(-1.4, 1.0, 13))
    axes[0, 1].set_yticks(np.linspace(-0.06, 0.06, 7))
    axes[0, 1].tick_params(axis='x', labelrotation=90)
    axes[0, 1].set_xlabel('Position (rad)')
    axes[0, 1].set_ylabel('Moment (Nm)')
    axes[0, 1].grid(True)


    axes[1, 0].set_title('Positions')
    axes[1, 0].plot(results[:, 0], label='Servo')
    axes[1, 0].plot(results[:, 1], label='Spring')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    axes[1, 1].set_title('Moments')
    axes[1, 1].plot(results[:, 3], label='Reaction')
    axes[1, 1].plot(results[:, 4], label='Spring')
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    fig.tight_layout()
    plt.savefig('fig_' + df.columns.values[c] + '.png')
    plt.show()

    results_df = pd.DataFrame(results, columns=['Servo Displacement (deg)', 'Spring Displacement (deg)', 'Spring Displacement (rad)', 'Tz (Nm)', 'Spring Moment (Nm)', 'Force (N)', 'Fx (N)', 'Fy (N)'])
    results_df.to_csv('results_' + df.columns.values[c] + '.csv')

print(df.columns.values)
print('Counters: ' + str(counters))
