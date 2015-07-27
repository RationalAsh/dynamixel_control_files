#!/usr/bin/python

import rconf
import pypot.robot
import pypot.primitive
#from pylab import *
import numpy as np
from numpy import sin, cos, pi
import time
from decimal import Decimal

#Angle limits for the motors
#KEY:
#L - Left     R - Right
#H - Hip      K - Knee
#S - Shoulder E - Elbow
#L at the end - Limit

LHL = [67.0, 123.0]
LKL = [-54.0, 111.0]

RHL = [67.0, 112.0]
RKL = [-89.0, 120.0]

LSL = [-120.0, -74.0]
LEL = [-150.0, 99.0]

RSL = [67.0, 115.0]
REL = [-86.0, 149.0]

#IDS of all the motors
MIDS = {
    'ls': 8,
    'le': 9,
    'rs': 5,
    're': 2,
    'lh': 4,
    'lk': 7,
    'rh': 6,
    'rk': 3
}


cuarc_config = {
    'controllers': {
        'cuarc_controller': {
            'sync_read': False,
            'attached_motors': ['left_leg', 'right_leg', 'right_arm', 'left_arm'],
            'port': '/dev/ttyACM0'
        }
    },
    'motorgroups': {
        'left_leg': ['lh', 'lk'],
        'right_leg': ['rh', 'rk'],
        'right_arm': ['rs', 're'],
        'left_arm': ['ls', 'le']
    },
    'motors': {
        'lh': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['lh'],
            'angle_limit': LHL,
            'offset': 0.0
        },
        'lk': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['lk'],
            'angle_limit': LKL,
            'offset': 0.0
        },
        'rh': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['rh'],
            'angle_limit': RHL,
            'offset': 0.0
        },
        'rk': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['rk'],
            'angle_limit': RKL,
            'offset': 0.0
        },
        'rs': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['rs'],
            'angle_limit': RSL,
            'offset': 0.0
        },
        're': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['re'],
            'angle_limit': REL,
            'offset': 0.0
        },
        'ls': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['ls'],
            'angle_limit': LSL,
            'offset': 0.0
        },
        'le': {
            'orientation': 'direct',
            'type': 'AX-12A',
            'id': MIDS['le'],
            'angle_limit': LEL,
            'offset': 0.0
        }
    }
}

def fmod(a, b):
    '''Handles floating point modulus'''
    if b==0:
        return
    return float(Decimal(str(a)) % Decimal(str(b)))

def kneefunc(time, amp=0.5, offset=0.5, freq=0.2, phase=0.0):
    '''
     This function generates the motion for the knee motor which controls the 
     bending of the knee. This function can be varied depending on what kind of
     motion we need.
    '''
    T = 1/float(freq)
    t = fmod(time, T)
    t = t-phase

    rval = 0
    if t<(0.125*T - phase):
        rval = amp*sin(2*pi*4*freq*t - pi/2 - phase) + offset
    elif t>=(0.125*T - phase) and t<(0.375*T - phase):
        rval =  amp + offset
    elif t>=(0.375*T - phase) and t<(0.5*T - phase):
        rval =  amp*cos(2*pi*4*freq*t + pi - phase) + offset
    elif t>=(0.5*T - phase):
        rval =  -amp + offset
    #print rval
    return rval

def kneefunc_shift(time, amp=0.5, offset=0.5, freq=0.2):
    '''
    This function generates the motion for the knee motor which controls the 
    bending of the knee. This function can be varied depending on what kind of
    motion we need.
    '''
    T = 1/float(freq)
    t = fmod(time, T)
    t = T - t
    rval = 0
    if t<0.125*T:
        rval = amp*sin(2*pi*4*freq*t - pi/2) + offset
    elif t>=0.125*T and t<0.375*T:
        rval =  amp + offset
    elif t>=0.375*T and t<0.5*T:
        rval =  amp*cos(2*pi*4*freq*t + pi) + offset
    elif t>=0.5*T:
        rval =  -amp + offset
    #print rval
    return rval

class trotPrimitive(pypot.primitive.LoopPrimitive):
    def update(self, freq=0.24):
        cuarc = self.robot
        rh = cuarc.rh
        rk = cuarc.rk
        lh = cuarc.lh
        lk = cuarc.lk
        rs = cuarc.rs
        re = cuarc.re
        ls = cuarc.ls
        le = cuarc.le

        a_rh = 0.5*(rh.upper_limit - rh.lower_limit)*\
               cos(2*pi*freq*self.elapsed_time) + \
               0.5*(rh.upper_limit + rh.lower_limit)
        
        a_rk = kneefunc_shift(self.elapsed_time, 
                              0.5*(rk.upper_limit - rk.lower_limit),
                              0.5*(rk.upper_limit + rk.lower_limit), freq)

        a_lh = 0.5*(lh.upper_limit - lh.lower_limit)*\
               cos(2*pi*freq*self.elapsed_time) + \
               0.5*(lh.upper_limit + lh.lower_limit) 
        
        a_lk = kneefunc(self.elapsed_time,
                        0.5*(lk.upper_limit - lk.lower_limit),
                        0.5*(lk.upper_limit + lk.lower_limit), freq)

        a_rs = 0.5*(rs.upper_limit - rs.lower_limit) * \
               cos(2*pi*freq*self.elapsed_time - pi) + \
               0.5*(rs.upper_limit + rs.lower_limit)
        
        a_re = kneefunc(self.elapsed_time,
                        0.5*(re.upper_limit - re.lower_limit),
                        0.5*(re.upper_limit + re.lower_limit), freq)

        a_ls = 0.5*(ls.upper_limit - ls.lower_limit) * \
               cos(2*pi*freq*self.elapsed_time - pi) + \
               0.5*(ls.upper_limit + ls.lower_limit)

        a_le = kneefunc(self.elapsed_time,
                              0.5*(le.upper_limit - le.lower_limit),
                              0.5*(le.upper_limit + le.lower_limit), freq)
        
        rh.goal_position = a_rh
        rk.goal_position = a_rk
        lh.goal_position = a_lh
        lk.goal_position = a_lk
        rs.goal_position = a_rs
        re.goal_position = a_re
        ls.goal_position = a_ls
        le.goal_position = a_le

class walkPrimitive(pypot.primitive.LoopPrimitive):
    def update(self, freq=0.15):
        cuarc = self.robot
        rh = cuarc.rh
        rk = cuarc.rk
        lh = cuarc.lh
        lk = cuarc.lk
        rs = cuarc.rs
        re = cuarc.re
        ls = cuarc.ls
        le = cuarc.le

        a_rh = 0.5*(rh.upper_limit - rh.lower_limit)*\
               cos(2*pi*freq*self.elapsed_time) + \
               0.5*(rh.upper_limit + rh.lower_limit)
        
        a_rk = kneefunc_shift(self.elapsed_time, 
                              0.5*(rk.upper_limit - rk.lower_limit),
                              0.5*(rk.upper_limit + rk.lower_limit), freq)

        a_lh = 0.5*(lh.upper_limit - lh.lower_limit)*\
               cos(2*pi*freq*self.elapsed_time) + \
               0.5*(lh.upper_limit + lh.lower_limit) 
        
        a_lk = kneefunc(self.elapsed_time,
                        0.5*(lk.upper_limit - lk.lower_limit),
                        0.5*(lk.upper_limit + lk.lower_limit), freq)

        a_rs = 0.5*(rs.upper_limit - rs.lower_limit) * \
               cos(2*pi*freq*self.elapsed_time - pi) + \
               0.5*(rs.upper_limit + rs.lower_limit)
        
        a_re = kneefunc(self.elapsed_time,
                        0.5*(re.upper_limit - re.lower_limit),
                        0.5*(re.upper_limit + re.lower_limit), freq)

        a_ls = 0.5*(ls.upper_limit - ls.lower_limit) * \
               cos(2*pi*freq*self.elapsed_time - pi) + \
               0.5*(ls.upper_limit + ls.lower_limit)

        a_le = kneefunc_shift(self.elapsed_time,
                              0.5*(le.upper_limit - le.lower_limit),
                              0.5*(le.upper_limit + le.lower_limit), freq)
        
        rh.goal_position = a_rh
        rk.goal_position = a_rk
        lh.goal_position = a_lh
        lk.goal_position = a_lk
        rs.goal_position = a_rs
        re.goal_position = a_re
        ls.goal_position = a_ls
        le.goal_position = a_le

class rlegPrimitive(pypot.primitive.Primitive):
    def run(self, freq=0.1):
        T = 1/freq
        cuarc = self.robot
        rh = cuarc.rh
        rk = cuarc.rk
        lh = cuarc.lh
        lk = cuarc.lk
        rs = cuarc.rs
        re = cuarc.re
        ls = cuarc.ls
        le = cuarc.le

        while self.elapsed_time < T:
            a_rh = 0.5*(rh.upper_limit - rh.lower_limit)*\
                   cos(2*pi*freq*self.elapsed_time) + \
                   0.5*(rh.upper_limit + rh.lower_limit)
        
            a_rk = kneefunc_shift(self.elapsed_time, 
                                  0.5*(rk.upper_limit - rk.lower_limit),
                                  0.5*(rk.upper_limit + rk.lower_limit), freq)
            rh.goal_position = a_rh
            rk.goal_position = a_rk
            time.sleep(0.05)

class llegPrimitive(pypot.primitive.Primitive):
    def run(self, freq=0.1):
        T = 1/freq
        cuarc = self.robot
        rh = cuarc.rh
        rk = cuarc.rk
        lh = cuarc.lh
        lk = cuarc.lk
        rs = cuarc.rs
        re = cuarc.re
        ls = cuarc.ls
        le = cuarc.le

        while self.elapsed_time < T:

            a_lh = 0.5*(lh.upper_limit - lh.lower_limit)*\
                   cos(2*pi*freq*self.elapsed_time) + \
                   0.5*(lh.upper_limit + lh.lower_limit) 
        
            a_lk = kneefunc(self.elapsed_time,
                            0.5*(lk.upper_limit - lk.lower_limit),
                            0.5*(lk.upper_limit + lk.lower_limit), freq)


            
            lh.goal_position = a_lh
            lk.goal_position = a_lk
            time.sleep(0.05)
F = 0.1
T = 1/F
cuarc = pypot.robot.from_config(cuarc_config)
trot = trotPrimitive(cuarc, 60)
walk = walkPrimitive(cuarc, 60)
rleg = rlegPrimitive(cuarc)
lleg = llegPrimitive(cuarc)

# while True:
#     rleg.start()
#     time.sleep(T)
#     lleg.start()

if __name__ == '__main__':
    pass
