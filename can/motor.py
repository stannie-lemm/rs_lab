import numpy as np 
from can import CAN_Bus
from time import time
from typing import NoReturn, Optional, Union, List, Tuple
from time import sleep
import multiprocessing as mp
from datetime import datetime as dt
import csv

class CAN_Device:
    dev_id: int
    bus: CAN_Bus 

    def __init__(self, dev_id: int, bus: CAN_Bus):
        self.dev_id = dev_id
        self.bus = bus
    
    def send(self, data: bytes) -> None:
        if not isinstance(data, bytes):
            raise TypeError('Data type must be bytes')
        self.bus.send_bytes(self.dev_id, data)
    
    def recive(self) -> bytes:
        can_id, _, data = self.bus.recive_frame()
        if can_id == self.dev_id:
            return data

class CAN_MotorStatus:
    encoder_bytes: bytes
    speed_bytes: bytes
    torque_bytes: bytes
    tmp_bytes: bytes

    encoder_data: int = None
    speed_data: int = None
    torque_data: int = None
    tmp_data: int = None
    time_data: float = None
    time_start: float

    log: List[Tuple[int, int, int, int, int]] = []

    turns: int = 0
    FULL_TURN: int = 2**14

    def __init__(self, log: bool = False, thresholds=(2**12, (2**12)*3)):
        self.LOG_SET = log
        self.thrshld = thresholds
        self.time_start = time()
    
    def save_state(self):
        self.log.append((self.encoder_data, self.speed_data, self.torque_data, self.tmp_data, self.turns, self.time_data))

    def process_data(self, data: bytes) -> None:
        self.encoder_bytes = data[6:]
        self.speed_bytes = data[4:6]
        encoder_temp = int.from_bytes(self.encoder_bytes,byteorder = 'little')
        speed_temp = int.from_bytes(self.speed_bytes,byteorder = 'little', signed=True)
        time_temp = time() - self.time_start
        if self.encoder_data is not None:
            self.process_encoder(encoder_temp, speed_temp, time_temp)
        self.speed_data = speed_temp
        self.encoder_data = encoder_temp
        self.time_data = time_temp
        self.save_state()
    
    def process_encoder(self, next_p, next_s, next_t) -> None:
        threshold_hi = self.thrshld[1]
        threshold_lo = self.thrshld[0]
        prev_p = self.encoder_data
        

        delta_t = (next_t - self.time_data)
        speed_av = self.speed_data # average speed between previous moment and next moment

        # if np.abs(speed_av) < 700:
        if prev_p > threshold_hi and next_p <= threshold_lo:
            self.turns += 1
        elif prev_p <= threshold_lo and next_p > threshold_hi:
            self.turns -= 1
        # else:
        #     delta_p = next_p - prev_p
        #     calc_s = delta_p/delta_t

        #     if calc_s < 0 and self.speed_data > 0:
        #         self.turns += 1
        #     elif calc_s > 0 and self.speed_data < 0:
        #         self.turns -= 1

        # current = data
        # # print(np.log(prev)/np.log(2), np.log(current)/np.log(2))
        # # print(prev < self.thrshld[0], current >= self.thrshld[1])
        # if prev > current and self.speed_data > 0:
        #     # print("a")
        #     self.turns += 1
        # elif prev < current and self.speed_data < 0:
        #     self.turns -= 1

    def get_data(self, v_filter=False):
        # velocity filtration
        vel = self.speed_data
        if v_filter:
            data = np.array(self.log[-20:])
            vel = np.mean(data[:, 1].flatten())
        return self.encoder_data + self.turns*self.FULL_TURN, vel
    
    def write_log(self):
        name = f'MOTOR_LOG.csv'
        with open(name, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(['Encoder', 'Speed', 'Torque', 'Temperature', 'Turns', 'Time'])
            writer.writerows(self.log)



class CAN_Motor(CAN_Device):
    _rad_mul = np.pi*2/2**14
    _deg_mul = 360/2**14
    MOTOR_STATUS_BASE: bytes = b'\x9C' + 7*b'\x00'
    TORQUE_CONTROL_BASE: bytes = b'\xA1'
    MOTOR_OFF_BASE: bytes = b'\x80' + 7*b'\x00'
    K_P: float = 0
    K_D: float = 0

    def __init__(self, dev_id: int, bus: CAN_Bus, log: bool = False):
        super().__init__(dev_id, bus)
        self.status = CAN_MotorStatus(log=log)

    def _generate_torque_frame(self, val: int, bound = 200):
        if bound > 2000:
            raise ValueError('Torque must be in range [-2000; 2000]')

        val = bound if val > bound else val
        val = -bound if val < -bound else val
        print(f'torque_val: {val}')
        val_b = val.to_bytes(2, 'little', signed=True)
        temp = self.TORQUE_CONTROL_BASE + 3*b'\x00' + val_b + 2*b'\x00'
        return temp
    
    def _send_n_rcv_torque(self, val: int, bound=200):
        frame = self._generate_torque_frame(val, bound=bound)
        self.send(frame)
        self.recv_status()
    
    def PD_control(self, Kp, Kd, q, qdot, bound = 200, v_filter=False):
        self.send(self.MOTOR_STATUS_BASE)
        self.recv_status()
        i = 0
        while True:
            if i %4 == 0:

                # sleep(0.05)
                qr, qdotr = self.status.get_data(v_filter=v_filter)
                qr = self._calc_degrees(qr)
                
                t = Kp*(q - qr) + Kd*(qdot - qdotr)
                print(f'pos: {qr}, vel: {qdotr}, torque: {t}', end='')
                self._send_n_rcv_torque(int(t), bound = bound)
            i += 1
            i = i//4
    
    def smooth_off(self, Kd=1):
        self.status.write_log()
        self.send(self.MOTOR_STATUS_BASE)
        self.recv_status()
        con = 2**14
        while self.status.get_data()[1] != 0 and np.abs((self.status.get_data()[0]/con*360)%360) > 3:
            q, qdotr = self.status.get_data()
            q = (q/con*360)%360
            t = 1*(0 - qdotr) + 10*(0 - q)
            self._send_n_rcv_torque(int(t))
        sleep(1)
        self.motor_off()
        sleep(1)
        

    
    # def smooth_off(self):
    #     torques = np.linspace(self.current_torque, 0, 5, dtype=np.int16)
    #     for t in torques:
    #         self._send_n_rcv_torque(int(t))
    #         sleep(0.1)
    #     while np.abs(self.current_vel) > 1:
    #         self.recv_data()
    #         self.get_inst_vel()
    
    def motor_off(self):
        self.send(self.MOTOR_OFF_BASE)
        
    def _calc_radians(self, val) -> float:
        return val*self._rad_mul
    
    def _calc_degrees(self, val) -> float:
        return val*self._deg_mul

    def _calc_abs_pos(self) -> int:
        return self.hi*self.current_turns + self.current_position
    
    def recv_status(self) -> None:
        self.status.process_data(self.recive())

    def get_position(self, units: Optional[str] = None) -> Union[float, int]:
        pos = self.status.get_data()[0]
        if units is None:
            return pos
        elif units is 'rad':
            return self._calc_radians(pos)
        elif units is 'deg':
            return self._calc_degrees(pos)
        else: 
            raise ValueError(f'No such unit {units}')

    def get_inst_vel(self) -> Optional[float]:
        return self.status.get_data()[1]

    # def get_inst_vel_calc(self, threshold_lo: int=2**4, threshold_hi: int = 2**12) -> Optional[float]:
    #     self.send(self.MOTOR_STATUS_BASE)
    #     data = self.recive()
    #     dt = time() - self.current_time
    #     if data is None:
    #         return

    #     parsed = int.from_bytes(data[6:],byteorder = 'little')
    #     ds = parsed - self.current_position
    #     return ds/dt

    # def get_inst_vel_filtered(self, threshold_lo: int=2**4, threshold_hi: int = 2**12) -> Optional[float]:
    #     self.recv_data()
    #     self._recv_pos()
    #     return np.mean(self.current_positions)
    
    
