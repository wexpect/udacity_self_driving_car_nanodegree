# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Loop over all frames in a Waymo Open Dataset file,
#                        detect and track objects and visualize results
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

##################
## Imports

## general package imports
import os
import sys
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import copy

## Add current working directory to path
sys.path.append(os.getcwd())

## Waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import WaymoDataFileReader, dataset_pb2, label_pb2

## 3d object detection
import student.objdet_pcl as pcl
import student.objdet_detect as det
import student.objdet_eval as eval

import misc.objdet_tools as tools 
from misc.helpers import save_object_to_file, load_object_from_file, make_exec_list

## Tracking
from student.filter import Filter
from student.trackmanagement import Trackmanagement
from student.association import Association
from student.measurements import Sensor, Measurement
from misc.evaluation import plot_tracks, plot_rmse, make_movie
import misc.params as params 
 
##################
## Set parameters and perform initializations


# options are 'darknet', 'fpn_resnet'
model = "darknet"
# model = "fpn_resnet"


# mid-term project
# sequence = "1"

# final project
sequence = "2"


## Select Waymo Open Dataset file and frame numbers
# data_filename = 'training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord' # Sequence 1
# # data_filename = 'training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord' # Sequence 2
# # data_filename = 'training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord' # Sequence 3

data_filenames = [
    '', # sequence 0
    'training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord', # Sequence 1
    'training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord', # Sequence 2
    'training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord', # Sequence 3
]
data_filename = data_filenames[int(sequence)]



# show_only_frames = [0, 200] # show only frames in interval for debugging
# mid-term project
# show_only_frames = [50, 51]

# final project
# show_only_frames = [150, 155]
show_only_frames = [150, 200]



## Prepare Waymo Open Dataset file for loading
data_fullpath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'dataset', data_filename) # adjustable path in case this script is called from another working directory



# for mid-term project
# results_fullpath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'results/' + model + '/results_sequence_' + sequence + '_' + model)

# for final project
results_fullpath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'results/Lidar_Detections_Tracking_Final_Project')



datafile = WaymoDataFileReader(data_fullpath)
datafile_iter = iter(datafile)  # initialize dataset iterator




## Initialize object detection
# configs_det = det.load_configs(model_name='fpn_resnet') # options are 'darknet', 'fpn_resnet'
configs_det = det.load_configs(model_name=model)

model_det = det.create_model(configs_det)



# False = use model-based detection, True = use groundtruth labels as objects
configs_det.use_labels_as_objects = False
# configs_det.use_labels_as_objects = True


# for final project
## Uncomment this setting to restrict the y-range in the final project
# configs_det.lim_y = [-25, 25]
configs_det.lim_y = [-5, 10]



## Initialize tracking
KF = Filter() # set up Kalman filter 
association = Association() # init data association
manager = Trackmanagement() # init track manager
lidar = None # init lidar sensor object
camera = None # init camera sensor object


np.random.seed(10) # make random values predictable



## Selective execution and visualization

# options are 'show_range_image', 'bev_from_pcl', 'detect_objects', 'validate_object_labels', 'measure_detection_performance'; options not in the list will be loaded from file
# exec_detection = ['bev_from_pcl', 'detect_objects', 'validate_object_labels', 'measure_detection_performance']
# exec_detection = ['bev_from_pcl']
# exec_detection = ['bev_from_pcl', 'detect_objects']

# for mid-term project
# exec_detection = ['bev_from_pcl', 'detect_objects', 'validate_object_labels', 'measure_detection_performance']

# for final project
exec_detection = []




# options are 'perform_tracking'
# for mid-term project
# exec_tracking = []

# for final project
exec_tracking = ['perform_tracking']




# options are 'show_range_image', 'show_pcl', 'show_bev', 'show_labels_in_image', 'show_objects_and_labels_in_bev', 'show_objects_in_bev_labels_in_camera', 'show_tracks', 'show_detection_performance', 'make_tracking_movie', 'show_objects_and_labels_in_bev_and_camera'
# exec_visualization = ['show_objects_in_bev_labels_in_camera']
# exec_visualization = ['show_detection_performance']
# exec_visualization = ['show_objects_in_bev_labels_in_camera', 'show_detection_performance']
# exec_visualization = ['show_objects_and_labels_in_bev_and_camera', 'show_detection_performance']

# for mid-term project
# exec_visualization = ['show_objects_and_labels_in_bev_and_camera']

# for final project
exec_visualization = ['show_tracks']



exec_list = make_exec_list(exec_detection, exec_tracking, exec_visualization)

vis_pause_time = 0  # set pause time between frames in ms (0 = stop between frames until key is pressed)


##################
## Perform detection & tracking over all selected frames

cnt_frame = 0 
all_labels = []
det_performance_all = []

if 'show_tracks' in exec_list:    
    fig, (ax2, ax) = plt.subplots(1, 2) # init track plot

while True:
    try:
        ## Get next frame from Waymo dataset
        frame = next(datafile_iter)
        if cnt_frame < show_only_frames[0]:
            cnt_frame = cnt_frame + 1
            continue
#         elif cnt_frame > show_only_frames[1]:
        elif cnt_frame >= show_only_frames[1]:
            print('reached end of selected frames')
            break
        
        print('------------------------------')
        print('processing frame #' + str(cnt_frame))

        #################################
        ## Perform 3D object detection

        ## Extract calibration data and front camera image from frame
        lidar_name = dataset_pb2.LaserName.TOP
        camera_name = dataset_pb2.CameraName.FRONT
        lidar_calibration = waymo_utils.get(frame.context.laser_calibrations, lidar_name)
        camera_calibration = waymo_utils.get(frame.context.camera_calibrations, camera_name)        
        # print('lidar_calibration\n', lidar_calibration)
        # print('camera_calibration\n', camera_calibration)

        # NOTE: output
        # lidar_calibration
        #     name: TOP
        #     beam_inclinations: -0.30748756489461426
        #     beam_inclinations: -0.2970812116814212
        #     beam_inclinations: -0.28595594089308407
        #     beam_inclinations: -0.2757298865611386
        #     beam_inclinations: -0.2650455276569803
        #     beam_inclinations: -0.2554331806229724
        #     beam_inclinations: -0.24605002585500046
        #     beam_inclinations: -0.23623723598159718
        #     beam_inclinations: -0.22678409144192635
        #     beam_inclinations: -0.2180910498144235
        #     beam_inclinations: -0.20810460304975886
        #     beam_inclinations: -0.1995358315129434
        #     beam_inclinations: -0.19120759074763627
        #     beam_inclinations: -0.18257185683077637
        #     beam_inclinations: -0.17411859007504038
        #     beam_inclinations: -0.16661091321694022
        #     beam_inclinations: -0.15847586917904977
        #     beam_inclinations: -0.1505908505475495
        #     beam_inclinations: -0.1433281288329944
        #     beam_inclinations: -0.1356896724438077
        #     beam_inclinations: -0.12874473913440854
        #     beam_inclinations: -0.12186047584228699
        #     beam_inclinations: -0.11497125307451461
        #     beam_inclinations: -0.10823392885070882
        #     beam_inclinations: -0.1022193935073108
        #     beam_inclinations: -0.09552688901866468
        #     beam_inclinations: -0.08952113257664318
        #     beam_inclinations: -0.08395698021916176
        #     beam_inclinations: -0.07853023383218627
        #     beam_inclinations: -0.07280710015937886
        #     beam_inclinations: -0.06790458616814088
        #     beam_inclinations: -0.06267055529836307
        #     beam_inclinations: -0.05780952053432031
        #     beam_inclinations: -0.053412373206688235
        #     beam_inclinations: -0.04906280792671747
        #     beam_inclinations: -0.044517259623126915
        #     beam_inclinations: -0.04047549990225274
        #     beam_inclinations: -0.036375824954014924
        #     beam_inclinations: -0.032602448767393266
        #     beam_inclinations: -0.028977311526227068
        #     beam_inclinations: -0.026182132264198632
        #     beam_inclinations: -0.022700660166731668
        #     beam_inclinations: -0.020041015122501893
        #     beam_inclinations: -0.016764648043835262
        #     beam_inclinations: -0.013741948246395186
        #     beam_inclinations: -0.01094997245789564
        #     beam_inclinations: -0.008463399915257463
        #     beam_inclinations: -0.0052617371486087805
        #     beam_inclinations: -0.0025192746574096425
        #     beam_inclinations: 0.00111439664469426
        #     beam_inclinations: 0.0036384736237624615
        #     beam_inclinations: 0.006621638853778977
        #     beam_inclinations: 0.009042329770240443
        #     beam_inclinations: 0.0121274334160959
        #     beam_inclinations: 0.01488915378670752
        #     beam_inclinations: 0.017985077053304988
        #     beam_inclinations: 0.021319223617011662
        #     beam_inclinations: 0.024159103729300524
        #     beam_inclinations: 0.02660515986667411
        #     beam_inclinations: 0.029626775195633392
        #     beam_inclinations: 0.03240145294822594
        #     beam_inclinations: 0.035505404585532974
        #     beam_inclinations: 0.038806828373769475
        #     beam_inclinations: 0.04166658785161581
        #     beam_inclination_min: -0.3126907415012108
        #     beam_inclination_max: 0.04309646759053898
        #     extrinsic
        #     {
        #         transform: -0.8530123627165663
        #         transform: -0.5218907060320562
        #         transform: -3.172210778948619e-06
        #         transform: 1.43
        #         transform: 0.5218861129965903
        #         transform: -0.8530048810279564
        #         transform: 0.004190227176684631
        #         transform: 0.0
        #         transform: -0.002189546530952744
        #         transform: 0.003572660051549892
        #         transform: 0.9999912209545365
        #         transform: 2.184
        #         transform: 0.0
        #         transform: 0.0
        #         transform: 0.0
        #         transform: 1.0
        #     }
        #
        # camera_calibration
        #     name: FRONT
        #     intrinsic: 2095.4997837421124
        #     intrinsic: 2095.4997837421124
        #     intrinsic: 944.8988009620841
        #     intrinsic: 640.2029570636614
        #     intrinsic: 0.047091611375765195
        #     intrinsic: -0.3598002261937365
        #     intrinsic: 0.001094881048492077
        #     intrinsic: -0.0002543425769804835
        #     intrinsic: 0.0
        #     extrinsic
        #     {
        #         transform: 0.9999563057051205
        #         transform: -0.004142937243963036
        #         transform: -0.008379901643826803
        #         transform: 1.5442226256119038
        #         transform: 0.004225690592669705
        #         transform: 0.9999422457406929
        #         transform: 0.009881736794455866
        #         transform: -0.023260141019037704
        #         transform: 0.008338478253413505
        #         transform: -0.009916715890478264
        #         transform: 0.9999160607402828
        #         transform: 2.1157612285419987
        #         transform: 0.0
        #         transform: 0.0
        #         transform: 0.0
        #         transform: 1.0
        #     }
        #     width: 1920
        #     height: 1280
        #     rolling_shutter_direction: RIGHT_TO_LEFT

        if 'load_image' in exec_list:
            print('extract image from frame')
            # NOTE: the image here is in BGR format
            image = tools.extract_front_camera_image(frame) 

        ## Compute lidar point-cloud from range image    
        if 'pcl_from_rangeimage' in exec_list:
            print('computing point-cloud from lidar range image')
            lidar_pcl = tools.pcl_from_range_image(frame, lidar_name)
        else:
            print('loading lidar point-cloud from result file')
            lidar_pcl = load_object_from_file(results_fullpath, data_filename, 'lidar_pcl', cnt_frame)
            
        ## Compute lidar birds-eye view (bev)
        if 'bev_from_pcl' in exec_list:
            print('computing birds-eye view from lidar pointcloud')
            lidar_bev = pcl.bev_from_pcl(lidar_pcl, configs_det)
        else:
            print('loading birds-eve view from result file')
            lidar_bev = load_object_from_file(results_fullpath, data_filename, 'lidar_bev', cnt_frame)

        ## 3D object detection
        if (configs_det.use_labels_as_objects==True):
            print('using groundtruth labels as objects')
            detections = tools.convert_labels_into_objects(frame.laser_labels, configs_det)
        else:
            if 'detect_objects' in exec_list:
                print('detecting objects in lidar pointcloud')   
                detections = det.detect_objects(lidar_bev, model_det, configs_det)
            else:
                print('loading detected objects from result file')
                # load different data for final project vs. mid-term project
                if 'perform_tracking' in exec_list:
                    # NOTE:
                    # detection = [class_id, x, y, z, h, w, l, yaw]
                    # e.g., [1, 49.57566010324578, 3.4130676169144483, 1.089919090270996, 2.06948, 1.8136293637125116, 4.577358772880152, 0.16567689]
                    detections = load_object_from_file(results_fullpath, data_filename, 'detections', cnt_frame)
                else:
                    detections = load_object_from_file(results_fullpath, data_filename, 'detections_' + configs_det.arch + '_' + str(configs_det.conf_thresh), cnt_frame)

        print('3D object detection\n', detections)

        ## Validate object labels
        if 'validate_object_labels' in exec_list:
            print("validating object labels")
            valid_label_flags = tools.validate_object_labels(frame.laser_labels, lidar_pcl, configs_det, 0 if configs_det.use_labels_as_objects == True else 10)
        else:
            print('loading object labels and validation from result file')
            valid_label_flags = load_object_from_file(results_fullpath, data_filename, 'valid_labels', cnt_frame)            

        ## Performance evaluation for object detection
        if 'measure_detection_performance' in exec_list:
            print('measuring detection performance')
            det_performance = eval.measure_detection_performance(detections, frame.laser_labels, valid_label_flags, configs_det.min_iou)
        else:
            print('loading detection performance measures from file')
            # load different data for final project vs. mid-term project
            if 'perform_tracking' in exec_list:
                det_performance = load_object_from_file(results_fullpath, data_filename, 'det_performance', cnt_frame)
            else:
                det_performance = load_object_from_file(results_fullpath, data_filename, 'det_performance_' + configs_det.arch + '_' + str(configs_det.conf_thresh), cnt_frame)

        # NOTE:
        # det_performance = [ious, center_devs, pos_negs]
        # iou, between detection and label
        # center_dev = [dist_x, dist_y, dist_z], distance between detection and label center
        # pos_neg = [all_positives, true_positives, false_negatives, false_positives]
        # e.g. [[0.7560755623637438], [[-0.052065038259996754, -0.2773427056683051, 0.1459045397737242]], [1, 1, 0, 0]]
        print('measure_detection_performance\n', det_performance)
        det_performance_all.append(det_performance) # store all evaluation results in a list for performance assessment at the end
        


        ## Visualization for object detection
        if 'show_range_image' in exec_list:
            img_range = pcl.show_range_image(frame, lidar_name)
            img_range = img_range.astype(np.uint8)
            cv2.imshow('range_image', img_range)
            cv2.waitKey(vis_pause_time)

        if 'show_pcl' in exec_list:
            pcl.show_pcl(lidar_pcl)

        if 'show_bev' in exec_list:
            tools.show_bev(lidar_bev, configs_det)  
            cv2.waitKey(vis_pause_time)          

        if 'show_labels_in_image' in exec_list:
            img_labels = tools.project_labels_into_camera(camera_calibration, image, frame.laser_labels, valid_label_flags, 0.5)
            cv2.imshow('img_labels', img_labels)
            cv2.waitKey(vis_pause_time)

        if 'show_objects_and_labels_in_bev' in exec_list:
            tools.show_objects_labels_in_bev(detections, frame.laser_labels, lidar_bev, configs_det)
            cv2.waitKey(vis_pause_time)         

        if 'show_objects_in_bev_labels_in_camera' in exec_list:
            tools.show_objects_in_bev_labels_in_camera(detections, lidar_bev, image, frame.laser_labels, valid_label_flags, camera_calibration, configs_det)
            cv2.waitKey(vis_pause_time)

        # NOTE: my code
        if 'show_objects_and_labels_in_bev_and_camera' in exec_list:
            tools.show_objects_and_labels_in_bev_and_camera(detections, lidar_bev, image, frame.laser_labels, valid_label_flags, camera_calibration, configs_det)
            cv2.waitKey(vis_pause_time)




        #################################
        ## Perform tracking
        if 'perform_tracking' in exec_list:
            # set up sensor objects
            if lidar is None:
                lidar = Sensor('lidar', lidar_calibration)
            if camera is None:
                camera = Sensor('camera', camera_calibration)
            
            # preprocess lidar detections
            meas_list_lidar = []
            for detection in detections:
                # NOTE:
                # detection = [class_id, x, y, z, h, w, l, yaw]
                # e.g., [[1, 49.57566010324578, 3.4130676169144483, 1.089919090270996, 2.06948, 1.8136293637125116, 4.577358772880152, 0.16567689]]

                # check if measurement lies inside specified range
                if detection[1] > configs_det.lim_x[0] and detection[1] < configs_det.lim_x[1] and detection[2] > configs_det.lim_y[0] and detection[2] < configs_det.lim_y[1]:
                    meas_list_lidar = lidar.generate_measurement(cnt_frame, detection[1:], meas_list_lidar)

            # preprocess camera detections
            meas_list_cam = []
            for label in frame.camera_labels[0].labels:
                if(label.type == label_pb2.Label.Type.TYPE_VEHICLE):
                
                    box = label.box
                    # use camera labels as measurements and add some random noise
                    z = [box.center_x, box.center_y, box.width, box.length]
                    z[0] = z[0] + np.random.normal(0, params.sigma_cam_i) 
                    z[1] = z[1] + np.random.normal(0, params.sigma_cam_j)
                    meas_list_cam = camera.generate_measurement(cnt_frame, z, meas_list_cam)
            
            # Kalman prediction
            print('track_list', manager.track_list)
            for track in manager.track_list:
                print('predict track', track.id)
                KF.predict(track)
                # track.set_t((cnt_frame - 1) * 0.1) # save next timestamp
                track.set_t((cnt_frame - 1) * params.dt)  # save next timestamp
                
            # associate all lidar measurements to all tracks
            print('associate lidar measurements to all tracks')
            association.associate_and_update(manager, meas_list_lidar, KF)
            
            # associate all camera measurements to all tracks
            print('associate camera measurements to all tracks')
            association.associate_and_update(manager, meas_list_cam, KF)
            
            # save results for evaluation
            result_dict = {}
            for track in manager.track_list:
                result_dict[track.id] = track

            manager.result_list.append(copy.deepcopy(result_dict))
            label_list = [frame.laser_labels, valid_label_flags]
            all_labels.append(label_list)
            
            # visualization
            if 'show_tracks' in exec_list:
                fig, ax, ax2 = plot_tracks(fig, ax, ax2, manager.track_list, meas_list_lidar, frame.laser_labels, 
                                        valid_label_flags, image, camera, configs_det)
                if 'make_tracking_movie' in exec_list:
                    # save track plots to file
                    fname = results_fullpath + '/tracking%03d.png' % cnt_frame
                    print('Saving frame', fname)
                    fig.savefig(fname)

        # increment frame counter
        cnt_frame = cnt_frame + 1    

    except StopIteration:
        # if StopIteration is raised, break from loop
        print("StopIteration has been raised\n")
        break


#################################
## Post-processing

## Evaluate object detection performance
if 'show_detection_performance' in exec_list:
    eval.compute_performance_stats(det_performance_all, configs_det)

## Plot RMSE for all tracks
if 'show_tracks' in exec_list:
    plot_rmse(manager, all_labels, configs_det)

## Make movie from tracking results    
if 'make_tracking_movie' in exec_list:
    make_movie(results_fullpath)
