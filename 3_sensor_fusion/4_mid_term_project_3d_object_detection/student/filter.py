# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state # process model dimension
        self.dt = params.dt # time increment
        self.q = params.q # process noise variable for Kalman filter Q
        # pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        dt = self.dt

        F = np.matrix([
            [1, 0,  0,  dt, 0,  0],
            [0, 1,  0,  0,  dt, 0],
            [0, 0,  1,  0,  0,  dt],
            [0, 0,  0,  1,  0,  0],
            [0, 0,  0,  0,  1,  0],
            [0, 0,  0,  0,  0,  1],
        ])
        print('F\n', F)
        return F
        # return 0
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        dt = self.dt
        q = self.q

        Q = np.matrix([
            [dt**3 * q / 3,     0,                  0,              dt**2 * q / 2,  0,              0],
            [0,                 dt**3 * q / 3,      0,              0,              dt**2 * q / 2,  0],
            [0,                             0,      dt**3 * q / 3,  0,              0,              dt** 2 * q / 2],
            [dt**2 * q / 2,     0,                  0,              dt * q,         0,              0],
            [0,                 dt**2 * q / 2,      0,              0,              dt * q,         0],
            [0,                 0,                  dt**2 * q / 2,  0,              0,              dt * q],
        ])
        print('Q\n', Q)
        return Q
        # return 0
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        print('predict start')

        F = self.F()
        Q = self.Q()

        x = track.x
        P = track.P
        print('x\n', x)
        print('P\n', P)

        x = F * x # state prediction
        P = F * P * F.transpose() + Q # covariance prediction

        print('x-\n', x)
        print('P-\n', P)

        track.set_x(x)
        track.set_P(P)

        # pass
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        print('update start')
        print(meas.sensor.name, 'z\n', meas.z)

        x = track.x
        print('x-\n', x)

        H = meas.sensor.get_H(x)
        print('H\n', H)

        gamma = self.gamma(track, meas)

        S = self.S(track, meas, H)

        P = track.P
        print('P-\n', P)

        K = P * H.transpose() * np.linalg.inv(S) # Kalman gain
        print('K\n', K)

        x = x + K * gamma # state update

        I = np.identity(self.dim_state)
        P = (I - K * H) * P # covariance update

        print('x+\n', x)
        print('P+\n', P)

        track.set_x(x)
        track.set_P(P)
        ############
        # END student code

        ############
        # Note: update width, length, height, rotation
        track.update_attributes(meas)

    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        x = track.x

        z = meas.z
        print('z\n', meas.z)

        hx = meas.sensor.get_hx(x)
        print('hx\n', hx)

        gamma = z - hx  # residual
        print('gamma\n', gamma)

        return gamma
        # return 0
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        P = track.P

        R = meas.R

        S = H * P * H.transpose() + R  # covariance of residual
        print('S\n', S)
        return S
        # return 0
        
        ############
        # END student code
        ############ 