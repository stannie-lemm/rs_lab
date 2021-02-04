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
# os.system('ifconfig > sas.log')
# protocol = {'motor_state':0x9C}
# message = b'\x9C' + 7*b'\x00'
# can_id = 0x141#0x142
# print('start sending message')
# # os.system('candump -l any,0~0,#FFFFFFFF')
# bus.send_bytes(can_id, message)
# # os.system('candump -l any,0~0,#FFFFFFFF')
# print('---end\n')
# os.system('ifconfig > sas1.log')
# print('start receiving message')
# _, _, data = bus.recive_frame()
# print('---end\n')
# encoder_data = int.from_bytes(data[6:],byteorder = 'little')
# encoder_data = data[7]<<8 | data[6]
# print(encoder_data)
pos = []
vel = []
time_stamps = []

try:
    # print(message)
    motor.send(b'\x9B' + 7*b'\x00')
    print(motor.recive())
    motor.send(b'\x88' + 7*b'\x00')
    print(motor.recive())
    while True:
        # bus.send_bytes(can_id, message)
        # _, _, data = bus.recive_frame()
        # encoder_data = int.from_bytes(data[6:],byteorder = 'little')
        # encoder_data = data[7]<<8 | data[6]
        # print(motor._generate_torque_frame(10))

        motor._send_n_rcv_torque(25)
        motor.recv_data()
        # pos.append(motor.get_position(units='deg'))
        # vel.append(motor.get_inst_vel())
        # time_stamps.append(time() - start)
    
        # print(f'position: {pos:10}, velocity: {vel:10}')

except KeyboardInterrupt:
    # arr1 = np.array(pos)
    # arr2 = np.array(vel)
    # arr3 = np.array(time_stamps)
    # np.save('pos.npy', arr1)
    # np.save('vel.npy', arr2)
    # np.save('time.npy', arr3)
    print('Exiting...')
    motor.send(b'\x9B' + 7*b'\x00')
    # motor._send_n_rcv_torque(0)
    motor.smooth_off()
    motor.motor_off()
    # motor.send(b'\x81' + 7*b'\x00')
    motor.recive()
    print('Motor off...')
    sleep(1)
    # bus.can_down()
# except:
#     bus.can_down()
