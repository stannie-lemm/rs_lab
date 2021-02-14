from can import CAN_Bus
from can.motor import CAN_Motor
import sys
import os
from time import time, sleep
import os
import numpy as np
from graphics import plot_data

bus = CAN_Bus(interface ='can0', bitrate=1000000)
motor = CAN_Motor(0x141, bus)
start = time()
pos = []
vel = []
time_stamps = []
desired_pos = 180
desired_vel = 0
k_p = 0
k_d = 0
k_p = 10
k_d = 1
bound = 2000
filter_flag = False
filter_flag = True

qs = [180]
qvs = [0]
Kps = [10, 1]
Kds = [10, 1]
Kis = [10, 1, 0.1]
bounds = [200, 50]
filters = [1, 20]
reducers = [1, 5]

tasks = []
for Q in qs:
    for QV in qvs:
        for KP in Kps:
            for KD in Kds:
                for KI in Kis:
                    for B in bounds:
                        for F in filters:
                            for R in reducers:
                                tasks.append({
                                    "Kp": KP,
                                    "Ki": KI,
                                    "Kd": KD,
                                    "P": Q,
                                    "V": QV,
                                    "R": R,
                                    "VF": F,
                                    "B": B
                                })


# print(tasks) 
motor.send(b'\x9B' + 7*b'\x00')
print(motor.recive())
motor.send(b'\x88' + 7*b'\x00')
print(motor.recive())
for task in tasks:
    motor.PD_control(0, 0)
    motor.reset_log()
    try:
        motor.PID_control(**task)
    except KeyboardInterrupt:
        motor._send_n_rcv_torque(0)
        plot_data(**task)
        motor.reset_log()

# except KeyboardInterrupt:
#     print('Exiting...')
#     motor.smooth_off()
#     print('Motor off...')
#     sleep(1)
#     bus.can_down()
