#!/usr/bin/env python

from ptg import PTG
from helpers import Vehicle, show_trajectory

def main():
	# NOTE: agent vehicle, s[speed, velocity, acceleration], d[speed, velocity, acceleration],
	vehicle = Vehicle([0,10,0, 0,0,0])

	predictions = {0: vehicle}
	target = 0

	# ego vehicle
	start_s = [10, 10, 0]
	start_d = [4, 0, 0]

	T = 5.0

	# TODO - change delta
	"""
     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at 
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal 
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a 
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"	
	"""
	# delta = [0, 0, 0, 0, 0 ,0]
	# delta = [-10, 0, 0, 0, 0 ,0]
	delta = [-20, 0, 0, 0, 0 ,0]

	best = PTG(start_s, start_d, target, delta, T, predictions)
	show_trajectory(best[0], best[1], best[2], vehicle)

if __name__ == "__main__":
	main()


"""
best trajectory  (array([10.        , 10.        ,  0.        , -1.55299434,  0.3921123 ,
       -0.02621403]), array([ 4.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.80959134e-02,
        4.05146076e-03,  1.52683820e-04]), 6.0)
        
cost for time_diff_cost is 	 0.0996679946249559
cost for s_diff_cost is 	 0.5272395476990153
cost for d_diff_cost is 	 1.37948018600749
cost for efficiency_cost is 	 0.49509970885422927
cost for max_jerk_cost is 	 0
cost for total_jerk_cost is 	 0.6847373169205335
collision_cost, nearest  3.5159861412341162
cost for collision_cost is 	 0.0
buffer_cost, nearest  3.5159861412341162
cost for buffer_cost is 	 1.207485331394606
cost for max_accel_cost is 	 0
cost for total_accel_cost is 	 0.9460928037144416

best cost  5.339802889215272
"""