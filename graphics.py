import numpy as np
import matplotlib.pyplot as plt
import csv

def plot_data(Kp: float, Kd: float, Ki: float, P: int, V: int, VF: int, B: int, R: int):
    """Plots motor position with respect to time. Used to test PID controller

    Args:
        Kp (float): Kp parameter
        Kd (float): Kd parameter
        Ki (float): Ki parameter
        P (int): Position (degrees units)
        V (int): Velocity (deg/sec)
        F (int): Velocity filter (parameter defining how much measurements are smoothed)
        B (int): Torque boundary
        R (int): Frequency reducer parameter (main freq/R)
    """
    fig = plt.figure()
    ax = plt.subplot(111)
    data = []
    with open('MOTOR_LOG.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        labels = spamreader.__next__()
        for row in spamreader:
            sas = []
            for j in row:
                if j != '':
                    sas.append(float(j))
                else:
                    sas.append(np.nan)
            data.append(sas)
    data = np.array(data)
    pos_raw = data[:, 0]
    turns = data[:, 4]
    pos = pos_raw/(2**14)*360 + turns*360
    pos = pos[::-1]
    time = data[:, 5]
    ax.plot(time, pos, label='Position')
    ax.plot(time, time*0 + P, 'r', label='Desired position')
    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('Position (deg)')
    ax.legend()
    ax.grid()
    fig.savefig(f'plots/KP_{Kp}_KD_{Kd}_KI_{Ki}_X_{P}_V_{V}_VF_{VF}_B_{B}_R_{R}.pdf', format='pdf')
    fig.clf()
    plt.close(fig)