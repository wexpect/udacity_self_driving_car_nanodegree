# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
        print('associate start')

        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # # the following only works for at most one track and one measurement
        # self.association_matrix = np.matrix([]) # reset matrix
        # self.unassigned_tracks = [] # reset lists
        # self.unassigned_meas = []
        #
        # if len(meas_list) > 0:
        #     self.unassigned_meas = [0]
        # if len(track_list) > 0:
        #     self.unassigned_tracks = [0]
        # if len(meas_list) > 0 and len(track_list) > 0:
        #     self.association_matrix = np.matrix([[0]])


        N = len(track_list)  # N tracks
        M = len(meas_list)  # M measurements
        self.unassigned_tracks = list(range(N))
        self.unassigned_meas = list(range(M))
        print('N', N, ', M', M)

        # initialize association matrix
        self.association_matrix = np.inf * np.ones((N, M))

        # loop over all tracks and all measurements to set up association matrix
        for i in range(N):
            track = track_list[i]
            for j in range(M):
                meas = meas_list[j]
                dist = self.MHD(track, meas, KF)
                if self.gating(dist, meas.sensor):
                    self.association_matrix[i, j] = dist
        print('association_matrix\n', self.association_matrix)

        print('associate end')

        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        # # the following only works for at most one track and one measurement
        # update_track = 0
        # update_meas = 0
        #
        # # remove from list
        # self.unassigned_tracks.remove(update_track)
        # self.unassigned_meas.remove(update_meas)
        # self.association_matrix = np.matrix([])
        #
        # return update_track, update_meas

        print('\nget_closest_track_and_meas')

        print('before')
        print('association_matrix\n', self.association_matrix)
        print('unassigned_tracks', self.unassigned_tracks)
        print('unassigned_meas', self.unassigned_meas)

        min_row, min_col = np.unravel_index(np.argmin(self.association_matrix, axis=None), self.association_matrix.shape)
        print('min_row', min_row)
        print('min_col', min_col)

        min_val = self.association_matrix[min_row, min_col]
        print('min_val', min_val)

        if min_val == np.inf:
            print('min_val is inf')
            return np.nan, np.nan

        self.association_matrix = np.delete(self.association_matrix, min_row, axis=0)
        self.association_matrix = np.delete(self.association_matrix, min_col, axis=1)

        track_ind = self.unassigned_tracks[min_row]
        meas_ind = self.unassigned_meas[min_col]
        print('track_ind', track_ind)
        print('meas_ind', meas_ind)

        self.unassigned_tracks = np.delete(self.unassigned_tracks, min_row)
        self.unassigned_meas = np.delete(self.unassigned_meas, min_col)

        print('\nafter')
        print('association_matrix\n', self.association_matrix)
        print('unassigned_tracks', self.unassigned_tracks)
        print('unassigned_meas', self.unassigned_meas)

        # return np.nan, np.nan
        return track_ind, meas_ind

        # ############
        # # END student code
        # ############

    def gating(self, MHD, sensor): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        degree_of_freedom = sensor.dim_meas
        val = chi2.ppf(params.gating_threshold, degree_of_freedom)
        print('MHD', MHD, 'val', val)

        if float(MHD) <= val:
            print('inside gate')
            return True
        else:
            print('out of gate')
            return False

        # pass
        
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
        # calc Mahalanobis distance
        print('calculate MHD')

        gamma = KF.gamma(track, meas)

        H = meas.sensor.get_H(track.x)
        S = KF.S(track, meas, H)

        MHD = gamma.transpose() * np.linalg.inv(S) * gamma  # Mahalanobis distance formula
        print('MHD', MHD)
        return MHD

        # pass
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            print('update associated tracks with measurements')

            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break

            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)

        print('after associate_and_update, track_manager.track_list contains')
        for track in manager.track_list:            
            print('track', track.id, 'score', track.score)