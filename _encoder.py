from can import CAN_Bus
from can.motor import CAN_Motor
import sys
import os
from time import time, sleep
import os
import numpy as np

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



try:
    motor.send(b'\x9B' + 7*b'\x00')
    print(motor.recive())
    motor.send(b'\x88' + 7*b'\x00')
    print(motor.recive())
    motor.PD_control(k_p, k_d, desired_pos, desired_vel, bound=bound, v_filter=filter_flag)

except KeyboardInterrupt:
    print('Exiting...')
    motor.smooth_off()
    print('Motor off...')
    sleep(1)
    bus.can_down()
