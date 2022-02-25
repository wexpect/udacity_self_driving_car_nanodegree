# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas, id):
        print('init track start')

        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        ############
        # TODO Step 2: initialization:
        # - replace fixed track initialization values by initialization of x and P based on 
        # unassigned measurement transformed from sensor to vehicle coordinates
        # - initialize track state and track score with appropriate values
        ############

        # step 1
        # self.x = np.matrix([
        #     [49.53980697],
        #     [ 3.41006279],
        #     [ 0.91790581],
        #     [ 0.        ],
        #     [ 0.        ],
        #     [ 0.        ]
        # ])
        # self.P = np.matrix([
        #     [9.0e-02, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00],
        #     [0.0e+00, 9.0e-02, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00],
        #     [0.0e+00, 0.0e+00, 6.4e-03, 0.0e+00, 0.0e+00, 0.0e+00],
        #     [0.0e+00, 0.0e+00, 0.0e+00, 2.5e+03, 0.0e+00, 0.0e+00],
        #     [0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 2.5e+03, 0.0e+00],
        #     [0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 0.0e+00, 2.5e+01]
        # ])
        #
        # self.state = 'confirmed'
        # self.score = 0


        # step 2
        self.x = np.zeros((6, 1))
        self.P = np.zeros((6, 6))

        print('z\n', meas.z)

        z_1 = np.ones((4, 1))
        z_1[0:3] = meas.z
        position_1 = meas.sensor.sens_to_veh * z_1
        self.x[0:3] = position_1[0:3]
        print('x\n', self.x)

        P_pos = M_rot * meas.R * M_rot.transpose()

        P_vel = np.matrix([
            [params.sigma_p44**2,   0,                      0],
            [0,                     params.sigma_p55**2,   0],
            [0,                     0,                      params.sigma_p66**2]
        ])

        self.P[0:3, 0:3] = P_pos
        self.P[3:6, 3:6] = P_vel
        print('P\n', self.P)

        self.state = 'initialized'
        self.score = 1. / params.window
        print('state', self.state)
        print('score', self.score)

        ############
        # END student code
        ############ 
               
        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

        print('init track end')

    def set_x(self, x):
        self.x = x
        
    def set_P(self, P):
        self.P = P  
        
    def set_t(self, t):
        self.t = t  
        
    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        # Note: only use lidar measurement
        if meas.sensor.name == 'lidar':
            c = params.weight_dim

            self.height = c * meas.height + (1 - c) * self.height
            self.width = c * meas.width + (1 - c) * self.width
            self.length = c * meas.length + (1 - c) * self.length

            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0] * np.cos(meas.yaw) + M_rot[0,1] * np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        
        
###################

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self):
        self.N = 0 # current number of tracks
        self.track_list = []
        self.last_id = -1
        self.result_list = []
        
    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):
        print('\nmanage_tracks start')

        print('len(track_list)', len(self.track_list))
        print('len(meas_list)', len(meas_list))

        print('unassigned_tracks', unassigned_tracks)
        print('unassigned_meas', unassigned_meas)


        ############
        # TODO Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big
        # (check params.py for parameters that might be helpful, but
        # feel free to define your own parameters)
        ############
        
        # decrease score for unassigned tracks
        print('\nfor unassigned tracks, decide if to update score')
        for i in unassigned_tracks:
            print('i', i)
            track = self.track_list[i]
            print('track.id', track.id)
            # check visibility    
            if meas_list: # if not empty
                '''
                NOTE: need to consider if track is inside FOV or not:
                
                1. if track is inside FOV, track score can change here, and track can be deleted below when P > max_P.
               
                2. if track is outside FOV, track score should remain the same, and track can be deleted below when P > max_P.
                After the object has disappeared from the visible range, it might take some time until the track is deleted. 
                This is okay because in theory the object is still there, so the track management tries to predict 
                the track further on. Just make sure that the track is deleted eventually.                                
                '''
                if meas_list[0].sensor.in_fov(track.x):
                    # your code goes here
                    # pass
                    print('decrease score, track', track.id)
                    print('before', track.score)
                    track.score = np.max([0, track.score - (1. / params.window)])
                    print('after', track.score)

        # delete old tracks
        print('\nfor unassigned tracks, decide if to delete track')
        deleted_tracks = []
        for i in unassigned_tracks:
            print('i', i)
            track = self.track_list[i]
            print('track.id', track.id)
            print('state', track.state)
            print('score', track.score)
            print('P[0,0]', track.P[0, 0])
            print('P[1,1]', track.P[1, 1])

            if track.state == 'confirmed':
                if (track.score < params.delete_threshold) | (track.P[0,0] > params.max_P) | (track.P[1,1] > params.max_P):
                    print('decide to delete')
                    deleted_tracks.append(track)
            if (track.state == 'initialized') | (track.state == 'tentative'):
                if (track.score < (1. / params.window)) | (track.P[0,0] > params.max_P) | (track.P[1,1] > params.max_P):
                    print('decide to delete')
                    deleted_tracks.append(track)

        for track in deleted_tracks:
            self.delete_track(track)

        ############
        # END student code
        ############ 
            
        # initialize new track with unassigned measurement
        # NOTE: here it only initialize new track with lidar measurements
        for j in unassigned_meas: 
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
                self.init_track(meas_list[j])

        print('manage_tracks end')
            
    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        self.track_list.remove(track)
        print('deleted track no.', track.id)

    def handle_updated_track(self, track):      
        ############
        # TODO Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############
        # pass
        print('handle_updated_track', track.id)

        print('increase score')
        print('before', track.score)
        track.score = np.min([1, track.score + (1. / params.window)])
        print('after', track.score)

        print('update state')
        print('before', track.state)
        if track.score >= params.confirmed_threshold:
            track.state = 'confirmed'
        elif track.score > (1. / params.window):
            track.state = 'tentative'
        print('after', track.state)

        ############
        # END student code
        ############