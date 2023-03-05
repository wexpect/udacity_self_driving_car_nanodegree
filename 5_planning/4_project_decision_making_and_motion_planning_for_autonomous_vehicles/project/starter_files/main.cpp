/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>


#include "json.hpp"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include "Eigen/QR"

#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"

#include <iostream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    } 
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Declare and initialize the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;


// NOTE: changed name from path_planner to behavior_and_motion_planner
// void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, 
//   double yaw, double velocity, State goal, bool is_junction, string tl_state, 
//   vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, 
//   vector<int>& best_spirals) {
void behavior_and_motion_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, 
  double yaw, double velocity, State goal, bool is_junction, string tl_state, 
  vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, 
  vector<int>& best_spirals) {
  
  std::cout << "\nbehavior_and_motion_planner start" << std::endl;

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;
  
  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], 
      x_points[x_points.size()-1], y_points[y_points.size()-1]);

  	ego_state.velocity.x = v_points[v_points.size()-1];	
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;
  	
  }

  Maneuver behavior = behavior_planner.get_active_maneuver();


  std::cout << "Goal before state_transition" << std::endl;
  std::cout << "location " << goal.location.x << ", " << goal.location.y << ", " << goal.location.z << std::endl;
  std::cout << "rotation " << goal.rotation.yaw << ", " << goal.rotation.pitch << ", " << goal.rotation.roll << std::endl;
  std::cout << "velocity " << goal.velocity.x << ", " << goal.velocity.y << ", " << goal.velocity.z << std::endl;
  std::cout << "acceleration " << goal.acceleration.x << ", " << goal.acceleration.y << ", " << goal.acceleration.z << std::endl;
  
  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  std::cout << "Goal after state_transition" << std::endl;
  std::cout << "location " << goal.location.x << ", " << goal.location.y << ", " << goal.location.z << std::endl;
  std::cout << "rotation " << goal.rotation.yaw << ", " << goal.rotation.pitch << ", " << goal.rotation.roll << std::endl;
  std::cout << "velocity " << goal.velocity.x << ", " << goal.velocity.y << ", " << goal.velocity.z << std::endl;
  std::cout << "acceleration " << goal.acceleration.x << ", " << goal.acceleration.y << ", " << goal.acceleration.z << std::endl;


  if(behavior == STOPPED){
  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];

  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);
  	}

  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory(
      spirals[i], desired_speed, ego_state, lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);  
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);
  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0) {
  	best_spiral_idx = best_spirals[best_spirals.size()-1];
    std::cout << "Find best spiral, idx " << best_spiral_idx << std::endl;
  }

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();

  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }

  std::cout << "behavior_and_motion_planner finish" << std::endl;
}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for(int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);  
	}
	obst_flag = true;
}

int main() {
  cout << "starting server" << endl;
  uWS::Hub h;

  h.onMessage( [](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {    
    auto s = hasData(data);

    if (s != "") {
      auto data = json::parse(s);

      vector<double> x_points = data["traj_x"];
      vector<double> y_points = data["traj_y"];
      vector<double> v_points = data["traj_v"];  // NOTE: velocity points
      double yaw = data["yaw"];  // NOTE: seems is default yaw
      double velocity = data["velocity"];  // NOTE: seems is default x.velocity
      double sim_time = data["time"];

      // NOTE: seems is goal, a new goal is passed in each time
      double waypoint_x = data["waypoint_x"];
      double waypoint_y = data["waypoint_y"];
      double waypoint_t = data["waypoint_t"];  // NOTE: theta, yaw
      bool is_junction = data["waypoint_j"];  // NOTE: seems is_goal_in_junction
      
      string tl_state = data["tl_state"];  // NOTE: traffic light status

      if(!have_obst){
        vector<double> x_obst = data["obst_x"];
        vector<double> y_obst = data["obst_y"];
        set_obst(x_obst, y_obst, obstacles, have_obst);
      }

      State goal;
      goal.location.x = waypoint_x;
      goal.location.y = waypoint_y;
      goal.rotation.yaw = waypoint_t;

      vector< vector<double> > spirals_x;
      vector< vector<double> > spirals_y;
      vector< vector<double> > spirals_v;
      vector<int> best_spirals;

      // path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, 
      //   spirals_x, spirals_y, spirals_v, best_spirals);          
      behavior_and_motion_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, 
        spirals_x, spirals_y, spirals_v, best_spirals);


      // Send control
      json msgJson;
      
      // NOTE: here assign constant throttle and steer in Control.
      // but not used in this project. see simulatorAPI.py L304
      msgJson["throttle"] = 0.25;  
      msgJson["steer"] = 0.0;
      
      msgJson["trajectory_x"] = x_points;
      msgJson["trajectory_y"] = y_points;
      msgJson["trajectory_v"] = v_points;
      msgJson["spirals_x"] = spirals_x;
      msgJson["spirals_y"] = spirals_y;
      msgJson["spirals_v"] = spirals_v;
      msgJson["spiral_idx"] = best_spirals;
      msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

      //  min point threshold before doing the update
      // for high update rate use 19 for slow update rate use 4 
      msgJson["update_point_thresh"] = 16;

      auto msg = msgJson.dump();
  
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);       
    }
  });
  
  
  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
  {
    cout << "Connected!!!" << endl;
  });

  
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

    int port = 4567;
    if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    } 
    else 
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }
    
}



/*
(venv) root@3d2e9e03eaf7:/home/workspace/nd013-c5-planning-starter/project# ./run_main.sh 
starting server
Listening to port 4567
Connected!!!
pygame 2.0.0 (SDL 2.0.12, python 3.6.9)
Hello from the pygame community. https://www.pygame.org/contribute.html
INFO: listening to server 127.0.0.1:2000

Welcome to CARLA path planning


ALSA lib confmisc.c:767:(parse_card) cannot find card '0'
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_card_driver returned error: No such file or directory
ALSA lib confmisc.c:392:(snd_func_concat) error evaluating strings
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_concat returned error: No such file or directory
ALSA lib confmisc.c:1246:(snd_func_refer) error evaluating name
ALSA lib conf.c:4528:(_snd_config_evaluate) function snd_func_refer returned error: No such file or directory
ALSA lib conf.c:5007:(snd_config_expand) Evaluate error: No such file or directory
ALSA lib pcm.c:2495:(snd_pcm_open_noupdate) Unknown PCM default
init keyboard control

behavior_and_motion_planner start
Goal before state_transition
location 73.5223, 7.61895, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
WARNING: Logging before InitGoogleLogging() is written to STDERR
I0212 14:53:34.076596  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 73.5223, 7.61895, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986559
spiral i 1, cost 0.963843
spiral i 2, cost 0.904869
spiral i 3, cost 0.762191
spiral i 4, cost 0.461769
spiral i 5, cost 0.00201167
spiral i 6, cost 0.461853
spiral i 7, cost 0.761702
spiral i 8, cost 0.904913
spiral i 9, cost 0.963887
spiral i 10, cost 0.986474
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 81.0962, 7.73209, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:37.950318  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 81.0962, 7.73209, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986454
spiral i 1, cost 0.963872
spiral i 2, cost 0.904894
spiral i 3, cost 0.761834
spiral i 4, cost 0.461792
spiral i 5, cost 7.43226e-05
spiral i 6, cost 0.461798
spiral i 7, cost 0.761978
spiral i 8, cost 0.90489
spiral i 9, cost 0.963862
spiral i 10, cost 0.986561
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 82.7802, 7.75724, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:39.783141  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 82.7802, 7.75724, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.98645
spiral i 1, cost 0.963871
spiral i 2, cost 0.904894
spiral i 3, cost 0.761852
spiral i 4, cost 0.461794
spiral i 5, cost 4.25099e-05
spiral i 6, cost 0.461796
spiral i 7, cost 0.761956
spiral i 8, cost 0.90489
spiral i 9, cost 0.963864
spiral i 10, cost 0.986561
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 84.4642, 7.7824, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:41.360538  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 84.4642, 7.7824, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986447
spiral i 1, cost 0.96387
spiral i 2, cost 0.904893
spiral i 3, cost 0.761871
spiral i 4, cost 0.461795
spiral i 5, cost 1.9164e-05
spiral i 6, cost 0.461794
spiral i 7, cost 0.761935
spiral i 8, cost 0.904891
spiral i 9, cost 0.963865
spiral i 10, cost 0.986435
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 86.1482, 7.80755, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:42.600159  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 86.1482, 7.80755, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986443
spiral i 1, cost 0.963869
spiral i 2, cost 0.904893
spiral i 3, cost 0.761888
spiral i 4, cost 0.461796
spiral i 5, cost 7.35382e-06
spiral i 6, cost 0.461793
spiral i 7, cost 0.761917
spiral i 8, cost 0.904891
spiral i 9, cost 0.963866
spiral i 10, cost 0.986439
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 87.8322, 7.83271, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:43.963738  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 87.8322, 7.83271, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986441
spiral i 1, cost 0.963868
spiral i 2, cost 0.904893
spiral i 3, cost 0.761901
spiral i 4, cost 0.461796
spiral i 5, cost 0.00102298
spiral i 6, cost 0.461792
spiral i 7, cost 0.761903
spiral i 8, cost 0.904891
spiral i 9, cost 0.963867
spiral i 10, cost 0.986441
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 89.5162, 7.85786, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:45.200256  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 89.5162, 7.85786, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986441
spiral i 1, cost 0.963867
spiral i 2, cost 0.904892
spiral i 3, cost 0.761904
spiral i 4, cost 0.461796
I0212 14:53:45.200667  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:53:45.200711  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
I0212 14:53:45.200740  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
spiral i 8, cost inf
spiral i 9, cost 0.963867
spiral i 10, cost 0.986442
Find best spiral, idx 4
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 91.2165, 7.88326, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:46.398566  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 91.2165, 7.88326, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986543
spiral i 1, cost 0.963907
spiral i 2, cost 0.905418
spiral i 3, cost 0.761379
I0212 14:53:46.399085  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:53:46.399168  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:53:46.399215  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
I0212 14:53:46.399257  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
I0212 14:53:46.399298  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:46.399344  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
spiral i 10, cost 0.986563
Find best spiral, idx 3
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 92.9436, 7.90906, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:47.633746  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 92.9436, 7.90906, 0
rotation -6.26825, 0, 0
velocity 2.99967, 0.04481, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986566
spiral i 1, cost 0.964079
spiral i 2, cost 0.905042
spiral i 3, cost 0.761423
I0212 14:53:47.634930  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:53:47.634990  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:53:47.635032  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
I0212 14:53:47.635265  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
I0212 14:53:47.635310  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:47.635473  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
spiral i 10, cost 0.986568
Find best spiral, idx 3
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 94.6259, 7.93419, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:48.895552  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 94.6259, 7.93419, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986573
spiral i 1, cost 0.963996
spiral i 2, cost 0.905045
spiral i 3, cost 0.761456
I0212 14:53:48.896049  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:53:48.896092  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:53:48.896142  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
I0212 14:53:48.896178  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
I0212 14:53:48.896214  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:48.896250  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
I0212 14:53:48.896282  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 10, cost inf
Find best spiral, idx 3
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 96.2821, 7.95893, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:50.236392  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 96.2821, 7.95893, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986624
spiral i 1, cost 0.963983
spiral i 2, cost 0.905071
spiral i 3, cost 0.76137
I0212 14:53:50.237835  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:53:50.237883  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:53:50.238055  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
I0212 14:53:50.238098  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
I0212 14:53:50.238137  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:50.238317  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
I0212 14:53:50.238361  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 10, cost inf
Find best spiral, idx 3
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 97.9344, 7.98361, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:51.441272  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 97.9344, 7.98361, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986616
spiral i 1, cost 0.963984
spiral i 2, cost 0.905094
spiral i 3, cost 0.761426
I0212 14:53:51.442237  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:53:51.442447  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:53:51.442482  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
I0212 14:53:51.442528  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
I0212 14:53:51.442924  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:51.442958  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
I0212 14:53:51.442988  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 10, cost inf
Find best spiral, idx 3
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 99.6479, 4.50882, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:52.758981  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 99.6479, 4.50882, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986456
spiral i 1, cost 0.963833
spiral i 2, cost 0.904808
spiral i 3, cost 0.760998
spiral i 4, cost 0.46174
spiral i 5, cost 0.000455985
spiral i 6, cost 0.462142
I0212 14:53:52.759656  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
I0212 14:53:52.759699  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:52.759747  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
I0212 14:53:52.759816  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 10, cost inf
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 101.342, 4.53412, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:53.959765  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 101.342, 4.53412, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986536
spiral i 1, cost 0.963881
spiral i 2, cost 0.904859
spiral i 3, cost 0.761484
spiral i 4, cost 0.461654
spiral i 5, cost 0.000247368
spiral i 6, cost 0.462823
I0212 14:53:53.960397  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 7, cost inf
I0212 14:53:53.960458  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:53.960494  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
I0212 14:53:53.960527  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 10, cost inf
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 103.007, 4.559, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:55.276733  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 103.007, 4.559, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.98655
spiral i 1, cost 0.963897
spiral i 2, cost 0.905282
spiral i 3, cost 0.761349
spiral i 4, cost 0.461758
spiral i 5, cost 0.000298043
spiral i 6, cost 0.461725
spiral i 7, cost 0.761177
I0212 14:53:55.277411  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 8, cost inf
I0212 14:53:55.277460  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 9, cost inf
I0212 14:53:55.277499  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 10, cost inf
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 104.664, 4.58375, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:56.454385  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 104.664, 4.58375, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986554
spiral i 1, cost 0.963906
spiral i 2, cost 0.905195
spiral i 3, cost 0.761336
spiral i 4, cost 0.461854
spiral i 5, cost 0.000272723
spiral i 6, cost 0.461624
spiral i 7, cost 0.761112
spiral i 8, cost 0.904587
spiral i 9, cost 0.963913
I0212 14:53:56.455188  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 10, cost inf
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 106.326, 4.60857, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:57.780952  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 106.326, 4.60857, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986555
spiral i 1, cost 0.96391
spiral i 2, cost 0.905198
spiral i 3, cost 0.761344
spiral i 4, cost 0.46191
spiral i 5, cost 6.56174e-05
spiral i 6, cost 0.464244
spiral i 7, cost 0.761096
spiral i 8, cost 0.904655
spiral i 9, cost 0.963907
spiral i 10, cost 0.986572
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 107.998, 4.63355, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:53:58.981246  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 107.998, 4.63355, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.98655
spiral i 1, cost 0.96391
spiral i 2, cost 0.905286
spiral i 3, cost 0.761358
spiral i 4, cost 0.461918
spiral i 5, cost 0.000157652
spiral i 6, cost 0.463002
spiral i 7, cost 0.761102
spiral i 8, cost 0.904762
spiral i 9, cost 0.963902
spiral i 10, cost 0.986565
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 109.678, 4.65864, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:00.198396  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 109.678, 4.65864, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986538
spiral i 1, cost 0.963907
spiral i 2, cost 0.904926
spiral i 3, cost 0.7614
spiral i 4, cost 0.461897
spiral i 5, cost 0.000192745
spiral i 6, cost 0.462218
spiral i 7, cost 0.761114
spiral i 8, cost 0.904826
spiral i 9, cost 0.963689
spiral i 10, cost 0.986562
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 111.362, 4.6838, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:01.509191  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 111.362, 4.6838, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986512
spiral i 1, cost 0.963899
spiral i 2, cost 0.904919
spiral i 3, cost 0.761512
spiral i 4, cost 0.461866
spiral i 5, cost 0.000157939
spiral i 6, cost 0.461881
spiral i 7, cost 0.76113
spiral i 8, cost 0.904857
spiral i 9, cost 0.963798
spiral i 10, cost 0.98656
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 113.048, 4.70897, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:02.670939  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 113.048, 4.70897, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986472
spiral i 1, cost 0.963885
spiral i 2, cost 0.904911
spiral i 3, cost 0.761713
spiral i 4, cost 0.461842
spiral i 5, cost 0.00158456
spiral i 6, cost 0.461774
spiral i 7, cost 0.762166
spiral i 8, cost 0.904872
spiral i 9, cost 0.963846
spiral i 10, cost 0.986559
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 114.732, 4.73413, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:03.895021  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 114.732, 4.73413, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986562
spiral i 1, cost 0.963867
spiral i 2, cost 0.904902
spiral i 3, cost 0.761956
spiral i 4, cost 0.461838
spiral i 5, cost 0.000371784
spiral i 6, cost 0.461755
spiral i 7, cost 0.761862
spiral i 8, cost 0.904882
spiral i 9, cost 0.963867
spiral i 10, cost 0.986454
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 116.416, 4.75928, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:05.077500  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 116.416, 4.75928, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986562
spiral i 1, cost 0.963852
spiral i 2, cost 0.904894
spiral i 3, cost 0.762161
spiral i 4, cost 0.461844
spiral i 5, cost 0.000686855
spiral i 6, cost 0.461763
spiral i 7, cost 0.761722
spiral i 8, cost 0.904889
spiral i 9, cost 0.963877
spiral i 10, cost 0.986477
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 118.099, 4.78443, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:06.351193  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 118.099, 4.78443, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986562
spiral i 1, cost 0.963843
spiral i 2, cost 0.904888
spiral i 3, cost 0.762271
spiral i 4, cost 0.461846
spiral i 5, cost 0.00115154
spiral i 6, cost 0.461775
spiral i 7, cost 0.761667
spiral i 8, cost 0.904894
spiral i 9, cost 0.963882
spiral i 10, cost 0.986486
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 119.782, 4.80957, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:07.551719  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 119.782, 4.80957, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986562
spiral i 1, cost 0.963841
spiral i 2, cost 0.904885
spiral i 3, cost 0.762281
spiral i 4, cost 0.461838
spiral i 5, cost 0.00124034
spiral i 6, cost 0.461785
spiral i 7, cost 0.76166
spiral i 8, cost 0.904897
spiral i 9, cost 0.963883
spiral i 10, cost 0.986486
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 121.465, 4.83471, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:08.695677  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 121.465, 4.83471, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963845
spiral i 2, cost 0.904884
spiral i 3, cost 0.762223
spiral i 4, cost 0.461823
spiral i 5, cost 0.000985771
spiral i 6, cost 0.461792
spiral i 7, cost 0.761683
spiral i 8, cost 0.904898
spiral i 9, cost 0.963882
spiral i 10, cost 0.986481
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 123.148, 4.85985, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:09.876940  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 123.148, 4.85985, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963851
spiral i 2, cost 0.904885
spiral i 3, cost 0.762136
spiral i 4, cost 0.461809
spiral i 5, cost 0.000608317
spiral i 6, cost 0.461795
spiral i 7, cost 0.761727
spiral i 8, cost 0.904898
spiral i 9, cost 0.96388
spiral i 10, cost 0.986473
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 124.832, 4.885, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:11.034775  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 124.832, 4.885, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost inf
I0212 14:54:11.037068  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:11.037595  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:11.038096  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:11.038554  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
spiral i 6, cost 0.461798
spiral i 7, cost 0.761781
spiral i 8, cost 0.904897
spiral i 9, cost 0.963876
spiral i 10, cost 0.986462
Find best spiral, idx 6
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 126.531, 4.91038, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:12.306238  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 126.531, 4.91038, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986563
I0212 14:54:12.306651  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:12.306699  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:12.306732  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:12.306768  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:12.306808  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:54:12.306888  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
spiral i 7, cost 0.761372
spiral i 8, cost 0.905376
spiral i 9, cost 0.963909
spiral i 10, cost 0.986546
Find best spiral, idx 7
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 128.256, 4.93614, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:13.466964  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 128.256, 4.93614, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986568
I0212 14:54:13.467352  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:13.467392  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:13.467434  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:13.467495  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:13.467551  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:54:13.467588  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
spiral i 7, cost 0.761428
spiral i 8, cost 0.905041
spiral i 9, cost 0.964074
spiral i 10, cost 0.986567
Find best spiral, idx 7
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 129.937, 4.96126, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:14.634337  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 129.937, 4.96126, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:14.634668  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:14.634708  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:14.634737  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:14.634774  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:14.634809  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:14.634824  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:54:14.634838  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
spiral i 7, cost 0.761462
spiral i 8, cost 0.905047
spiral i 9, cost 0.963995
spiral i 10, cost 0.986574
Find best spiral, idx 7
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 131.594, 4.98601, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:15.899209  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 131.594, 4.98601, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:15.900149  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:15.901546  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:15.901597  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:15.902109  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:15.902154  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:15.902590  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:54:15.902648  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
spiral i 7, cost 0.761387
spiral i 8, cost 0.905072
spiral i 9, cost 0.963984
spiral i 10, cost 0.986624
Find best spiral, idx 7
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 133.247, 5.01071, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:17.037278  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 133.247, 5.01071, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:17.037618  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:17.037680  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:17.037717  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:17.037748  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:17.037781  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:17.037818  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
I0212 14:54:17.037869  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 6, cost inf
spiral i 7, cost 0.761446
spiral i 8, cost 0.905094
spiral i 9, cost 0.963984
spiral i 10, cost 0.986616
Find best spiral, idx 7
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 134.857, 8.53515, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:18.280634  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 134.857, 8.53515, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:18.280997  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:18.281083  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:18.281141  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
spiral i 3, cost 0.76186
spiral i 4, cost 0.46214
spiral i 5, cost 0.000458579
spiral i 6, cost 0.461749
spiral i 7, cost 0.760997
spiral i 8, cost 0.904807
spiral i 9, cost 0.96383
spiral i 10, cost 0.986451
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 136.551, 8.56046, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:19.415555  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 136.551, 8.56046, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:19.416327  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:19.417687  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:19.417749  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
spiral i 3, cost 0.761277
spiral i 4, cost 0.462768
spiral i 5, cost 0.000244966
spiral i 6, cost 0.461653
spiral i 7, cost 0.761492
spiral i 8, cost 0.904858
spiral i 9, cost 0.963881
spiral i 10, cost 0.986535
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 138.218, 8.58535, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:20.576288  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 138.218, 8.58535, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:20.576619  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:20.576685  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
spiral i 2, cost 0.904704
spiral i 3, cost 0.76118
spiral i 4, cost 0.464461
spiral i 5, cost 0.000282418
spiral i 6, cost 0.461756
spiral i 7, cost 0.761351
spiral i 8, cost 0.905289
spiral i 9, cost 0.963897
spiral i 10, cost 0.986549
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 139.875, 8.61011, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:21.825732  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 139.875, 8.61011, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.98658
spiral i 1, cost 0.963913
spiral i 2, cost 0.904597
spiral i 3, cost 0.761115
spiral i 4, cost 0.461627
spiral i 5, cost 0.000259691
spiral i 6, cost 0.461852
spiral i 7, cost 0.761337
spiral i 8, cost 0.905199
spiral i 9, cost 0.963905
spiral i 10, cost 0.986554
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 141.537, 8.63494, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:22.966631  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 141.537, 8.63494, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986572
spiral i 1, cost 0.963907
spiral i 2, cost 0.904659
spiral i 3, cost 0.761098
spiral i 4, cost 0.464209
spiral i 5, cost 6.31114e-05
spiral i 6, cost 0.461908
spiral i 7, cost 0.761344
spiral i 8, cost 0.905201
spiral i 9, cost 0.96391
spiral i 10, cost 0.986554
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 143.209, 8.65991, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:24.212692  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 143.209, 8.65991, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986565
spiral i 1, cost 0.963902
spiral i 2, cost 0.904763
spiral i 3, cost 0.761103
spiral i 4, cost 0.462996
spiral i 5, cost 0.000156548
spiral i 6, cost 0.461917
spiral i 7, cost 0.761358
spiral i 8, cost 0.905288
spiral i 9, cost 0.96391
spiral i 10, cost 0.98655
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 144.889, 8.685, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:25.354998  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 144.889, 8.685, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986562
spiral i 1, cost 0.963688
spiral i 2, cost 0.904826
spiral i 3, cost 0.761114
spiral i 4, cost 0.462221
spiral i 5, cost 0.000191787
spiral i 6, cost 0.461896
spiral i 7, cost 0.761399
spiral i 8, cost 0.904926
spiral i 9, cost 0.963907
spiral i 10, cost 0.986538
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 146.573, 8.71016, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:26.489693  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 146.573, 8.71016, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.98656
spiral i 1, cost 0.963797
spiral i 2, cost 0.904857
spiral i 3, cost 0.76113
spiral i 4, cost 0.461883
spiral i 5, cost 0.000157726
spiral i 6, cost 0.461865
spiral i 7, cost 0.76151
spiral i 8, cost 0.904919
spiral i 9, cost 0.963899
spiral i 10, cost 0.986513
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 148.259, 8.73533, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:27.625433  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 148.259, 8.73533, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986559
spiral i 1, cost 0.963845
spiral i 2, cost 0.904872
spiral i 3, cost 0.762174
spiral i 4, cost 0.461775
spiral i 5, cost 0.00161832
spiral i 6, cost 0.461842
spiral i 7, cost 0.761709
spiral i 8, cost 0.904911
spiral i 9, cost 0.963885
spiral i 10, cost 0.986473
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 149.943, 8.76049, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:28.876397  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 149.943, 8.76049, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986454
spiral i 1, cost 0.963867
spiral i 2, cost 0.904882
spiral i 3, cost 0.761867
spiral i 4, cost 0.461755
spiral i 5, cost 0.000369147
spiral i 6, cost 0.461838
spiral i 7, cost 0.76195
spiral i 8, cost 0.904902
spiral i 9, cost 0.963868
spiral i 10, cost 0.986562
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 151.627, 8.78564, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:30.026959  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 151.627, 8.78564, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986477
spiral i 1, cost 0.963877
spiral i 2, cost 0.904889
spiral i 3, cost 0.761725
spiral i 4, cost 0.461763
spiral i 5, cost 0.000662809
spiral i 6, cost 0.461843
spiral i 7, cost 0.762155
spiral i 8, cost 0.904894
spiral i 9, cost 0.963852
spiral i 10, cost 0.986562
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 153.31, 8.81079, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:31.160369  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 153.31, 8.81079, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986485
spiral i 1, cost 0.963882
spiral i 2, cost 0.904894
spiral i 3, cost 0.761669
spiral i 4, cost 0.461775
spiral i 5, cost 0.00112482
spiral i 6, cost 0.461846
spiral i 7, cost 0.762266
spiral i 8, cost 0.904888
spiral i 9, cost 0.963843
spiral i 10, cost 0.986562
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 154.993, 8.83593, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:32.297410  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 154.993, 8.83593, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986486
spiral i 1, cost 0.963883
spiral i 2, cost 0.904897
spiral i 3, cost 0.761661
spiral i 4, cost 0.461785
spiral i 5, cost 0.00122131
spiral i 6, cost 0.461838
spiral i 7, cost 0.762278
spiral i 8, cost 0.904885
spiral i 9, cost 0.963842
spiral i 10, cost 0.986562
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 156.676, 8.86107, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:33.554631  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 156.676, 8.86107, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986481
spiral i 1, cost 0.963882
spiral i 2, cost 0.904898
spiral i 3, cost 0.761684
spiral i 4, cost 0.461791
spiral i 5, cost 0.000976676
spiral i 6, cost 0.461823
spiral i 7, cost 0.762221
spiral i 8, cost 0.904884
spiral i 9, cost 0.963845
spiral i 10, cost 0.986561
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 158.359, 8.88622, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:34.706424  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 158.359, 8.88622, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986473
spiral i 1, cost 0.96388
spiral i 2, cost 0.904898
spiral i 3, cost 0.761727
spiral i 4, cost 0.461795
spiral i 5, cost 0.00060611
spiral i 6, cost 0.461809
spiral i 7, cost 0.762136
spiral i 8, cost 0.904885
spiral i 9, cost 0.963851
spiral i 10, cost 0.986561
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 160.043, 8.91137, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:35.808104  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 160.043, 8.91137, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986463
spiral i 1, cost 0.963876
spiral i 2, cost 0.904897
spiral i 3, cost 0.761781
spiral i 4, cost 0.461797
spiral i 5, cost 0.000289743
spiral i 6, cost 0.461798
spiral i 7, cost 0.762049
spiral i 8, cost 0.904886
spiral i 9, cost 0.963857
spiral i 10, cost 0.986561
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 161.727, 8.93652, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:36.942431  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 161.727, 8.93652, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986452
spiral i 1, cost 0.963873
spiral i 2, cost 0.904896
spiral i 3, cost 0.761836
spiral i 4, cost 0.461799
spiral i 5, cost 9.99474e-05
spiral i 6, cost 0.461791
spiral i 7, cost 0.761976
spiral i 8, cost 0.904888
spiral i 9, cost 0.963862
spiral i 10, cost 0.986561
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 163.411, 8.96167, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:38.178282  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 163.411, 8.96167, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986444
spiral i 1, cost 0.963869
spiral i 2, cost 0.904894
spiral i 3, cost 0.761884
spiral i 4, cost 0.4618
spiral i 5, cost 2.42751e-05
spiral i 6, cost 0.461788
spiral i 7, cost 0.761922
spiral i 8, cost 0.90489
spiral i 9, cost 0.963866
spiral i 10, cost 0.986439
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 165.095, 8.98683, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:39.313223  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 165.095, 8.98683, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986437
spiral i 1, cost 0.963867
spiral i 2, cost 0.904893
spiral i 3, cost 0.761919
spiral i 4, cost 0.4618
spiral i 5, cost 1.06685e-05
spiral i 6, cost 0.461788
spiral i 7, cost 0.761886
spiral i 8, cost 0.904891
spiral i 9, cost 0.963868
spiral i 10, cost 0.986445
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 166.779, 9.01198, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:40.448693  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 166.779, 9.01198, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963865
spiral i 2, cost 0.904892
spiral i 3, cost 0.761941
spiral i 4, cost 0.4618
spiral i 5, cost 1.90883e-05
spiral i 6, cost 0.461789
spiral i 7, cost 0.761866
spiral i 8, cost 0.904892
spiral i 9, cost 0.96387
spiral i 10, cost 0.986448
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 168.463, 9.03714, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:41.587366  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 168.463, 9.03714, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963864
spiral i 2, cost 0.904891
spiral i 3, cost 0.761949
spiral i 4, cost 0.461799
spiral i 5, cost 2.738e-05
spiral i 6, cost 0.46179
spiral i 7, cost 0.761859
spiral i 8, cost 0.904893
spiral i 9, cost 0.96387
spiral i 10, cost 0.986449
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 170.147, 9.06229, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:42.709270  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 170.147, 9.06229, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:42.709784  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:42.711196  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:42.711302  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:42.711349  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:42.711884  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
spiral i 5, cost 2.6786e-05
spiral i 6, cost 0.461791
spiral i 7, cost 0.76186
spiral i 8, cost 0.904893
spiral i 9, cost 0.96387
spiral i 10, cost 0.986449
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 171.831, 9.08745, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:43.929948  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 171.831, 9.08745, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:43.930295  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:43.930627  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:43.930673  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:43.930860  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:43.930907  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:43.931058  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
spiral i 6, cost 0.461793
spiral i 7, cost 0.761867
spiral i 8, cost 0.904893
spiral i 9, cost 0.96387
spiral i 10, cost 0.986448
Find best spiral, idx 6
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 173.531, 9.11285, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:45.058562  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 173.531, 9.11285, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:45.059118  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:45.059602  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:45.059667  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:45.059724  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:45.059917  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:45.059978  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
spiral i 6, cost 0.461895
spiral i 7, cost 0.761375
spiral i 8, cost 0.9054
spiral i 9, cost 0.963908
spiral i 10, cost 0.986544
Find best spiral, idx 6
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 175.219, 9.13806, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:46.163946  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 175.219, 9.13806, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:46.164323  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:46.164369  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:46.164403  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:46.164429  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:46.164448  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:46.164463  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
spiral i 6, cost 0.462024
spiral i 7, cost 0.761368
spiral i 8, cost 0.905134
spiral i 9, cost 0.963919
spiral i 10, cost 0.986559
Find best spiral, idx 6
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 176.898, 9.16314, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:47.314924  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 176.898, 9.16314, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:47.315292  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:47.315356  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:47.315385  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:47.315418  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:47.315443  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:47.315457  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
spiral i 6, cost 0.46211
spiral i 7, cost 0.761403
spiral i 8, cost 0.905072
spiral i 9, cost 0.963927
spiral i 10, cost 0.986564
Find best spiral, idx 6
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 178.573, 9.18816, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:48.541747  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 178.573, 9.18816, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:48.542079  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:48.542577  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:48.542827  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:48.542932  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:48.543407  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:48.543520  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
spiral i 6, cost 0.462165
spiral i 7, cost 0.761431
spiral i 8, cost 0.905058
spiral i 9, cost 0.963933
spiral i 10, cost 0.986566
Find best spiral, idx 6
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 180.25, 9.21321, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:49.636566  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 180.25, 9.21321, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:49.636883  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:49.637383  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:49.637586  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:49.637650  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:49.637696  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
I0212 14:54:49.637874  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 5, cost inf
spiral i 6, cost 0.462203
spiral i 7, cost 0.761446
spiral i 8, cost 0.905062
spiral i 9, cost 0.963935
spiral i 10, cost 0.986567
Find best spiral, idx 6
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 181.93, 9.2383, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:50.782662  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 181.93, 9.2383, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:50.783255  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:50.784966  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:50.785027  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:50.785080  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
I0212 14:54:50.785140  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 4, cost inf
spiral i 5, cost 0.000294899
spiral i 6, cost 0.462218
spiral i 7, cost 0.761449
spiral i 8, cost 0.905077
spiral i 9, cost 0.963936
spiral i 10, cost 0.986567
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 183.635, 9.26377, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:51.964172  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 183.635, 9.26377, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:51.964537  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:51.965035  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
I0212 14:54:51.965083  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 2, cost inf
I0212 14:54:51.965265  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 3, cost inf
spiral i 4, cost 0.461969
spiral i 5, cost 0.00039402
spiral i 6, cost 0.462056
spiral i 7, cost 0.76149
spiral i 8, cost 0.904963
spiral i 9, cost 0.963919
spiral i 10, cost 0.986529
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 185.331, 9.28911, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:53.082718  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 185.331, 9.28911, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
I0212 14:54:53.083043  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 0, cost inf
I0212 14:54:53.083101  2964 cost_functions.cpp:45]  ***** COLLISION DETECTED *********
spiral i 1, cost inf
spiral i 2, cost 0.904855
spiral i 3, cost 0.761786
spiral i 4, cost 0.461672
spiral i 5, cost 0.000213154
spiral i 6, cost 0.462001
spiral i 7, cost 0.762267
spiral i 8, cost 0.904928
spiral i 9, cost 0.96386
spiral i 10, cost 0.986567
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 187.014, 9.31425, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:54.217778  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 187.014, 9.31425, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.98653
spiral i 1, cost 0.963888
spiral i 2, cost 0.904881
spiral i 3, cost 0.761464
spiral i 4, cost 0.461721
spiral i 5, cost 6.68263e-05
spiral i 6, cost 0.462264
spiral i 7, cost 0.761221
spiral i 8, cost 0.904882
spiral i 9, cost 0.963732
spiral i 10, cost 0.986567
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 188.69, 9.33928, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:55.294562  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 188.69, 9.33928, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.98654
spiral i 1, cost 0.963897
spiral i 2, cost 0.904899
spiral i 3, cost 0.761389
spiral i 4, cost 0.461787
spiral i 5, cost 2.28767e-05
spiral i 6, cost 0.462546
spiral i 7, cost 0.761178
spiral i 8, cost 0.904839
spiral i 9, cost 0.963622
spiral i 10, cost 0.986567
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 190.365, 9.3643, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:56.495879  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 190.365, 9.3643, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986542
spiral i 1, cost 0.963902
spiral i 2, cost 0.90491
spiral i 3, cost 0.761378
spiral i 4, cost 0.461831
spiral i 5, cost 8.65279e-05
spiral i 6, cost 0.462516
spiral i 7, cost 0.761152
spiral i 8, cost 0.904826
spiral i 9, cost 0.963617
spiral i 10, cost 0.986565
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 192.042, 9.38936, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:57.604976  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 192.042, 9.38936, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986537
spiral i 1, cost 0.963903
spiral i 2, cost 0.904914
spiral i 3, cost 0.761397
spiral i 4, cost 0.461848
spiral i 5, cost 0.000125988
spiral i 6, cost 0.462267
spiral i 7, cost 0.761141
spiral i 8, cost 0.904837
spiral i 9, cost 0.963684
spiral i 10, cost 0.986563
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 193.723, 9.41446, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:58.683611  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 193.723, 9.41446, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986526
spiral i 1, cost 0.9639
spiral i 2, cost 0.904914
spiral i 3, cost 0.761449
spiral i 4, cost 0.461846
spiral i 5, cost 0.000131981
spiral i 6, cost 0.462019
spiral i 7, cost 0.761141
spiral i 8, cost 0.904853
spiral i 9, cost 0.963759
spiral i 10, cost 0.986562
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 195.406, 9.43961, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:54:59.906399  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 195.406, 9.43961, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986506
spiral i 1, cost 0.963894
spiral i 2, cost 0.904911
spiral i 3, cost 0.761544
spiral i 4, cost 0.461835
spiral i 5, cost 0.00435445
spiral i 6, cost 0.461866
spiral i 7, cost 0.761148
spiral i 8, cost 0.904867
spiral i 9, cost 0.963811
spiral i 10, cost 0.986561
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 197.089, 9.46474, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:00.989933  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 197.089, 9.46474, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986479
spiral i 1, cost 0.963885
spiral i 2, cost 0.904907
spiral i 3, cost 0.761683
spiral i 4, cost 0.461824
spiral i 5, cost 0.00143656
spiral i 6, cost 0.461795
spiral i 7, cost 0.76222
spiral i 8, cost 0.904876
spiral i 9, cost 0.963843
spiral i 10, cost 0.98656
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 198.773, 9.48989, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:02.096618  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 198.773, 9.48989, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986449
spiral i 1, cost 0.963874
spiral i 2, cost 0.904901
spiral i 3, cost 0.761842
spiral i 4, cost 0.46182
spiral i 5, cost 0.000293416
spiral i 6, cost 0.461772
spiral i 7, cost 0.761971
spiral i 8, cost 0.904883
spiral i 9, cost 0.963861
spiral i 10, cost 0.986433
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 200.457, 9.51505, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:03.209046  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 200.457, 9.51505, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986562
spiral i 1, cost 0.963864
spiral i 2, cost 0.904896
spiral i 3, cost 0.761983
spiral i 4, cost 0.46182
spiral i 5, cost 0.000146903
spiral i 6, cost 0.461771
spiral i 7, cost 0.761834
spiral i 8, cost 0.904888
spiral i 9, cost 0.96387
spiral i 10, cost 0.986457
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 202.141, 9.5402, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:04.385965  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 202.141, 9.5402, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986562
spiral i 1, cost 0.963857
spiral i 2, cost 0.904892
spiral i 3, cost 0.762075
spiral i 4, cost 0.461821
spiral i 5, cost 0.000319864
spiral i 6, cost 0.461776
spiral i 7, cost 0.761767
spiral i 8, cost 0.904892
spiral i 9, cost 0.963875
spiral i 10, cost 0.986468
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 203.824, 9.56535, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:05.481071  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 203.824, 9.56535, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963854
spiral i 2, cost 0.904889
spiral i 3, cost 0.762111
spiral i 4, cost 0.461818
spiral i 5, cost 0.00044245
spiral i 6, cost 0.461782
spiral i 7, cost 0.761743
spiral i 8, cost 0.904894
spiral i 9, cost 0.963877
spiral i 10, cost 0.986471
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 205.508, 9.5905, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:06.597774  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 205.508, 9.5905, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963854
spiral i 2, cost 0.904888
spiral i 3, cost 0.762101
spiral i 4, cost 0.461812
spiral i 5, cost 0.000419329
spiral i 6, cost 0.461788
spiral i 7, cost 0.761748
spiral i 8, cost 0.904896
spiral i 9, cost 0.963878
spiral i 10, cost 0.98647
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 207.192, 9.61565, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:07.682883  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 207.192, 9.61565, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963856
spiral i 2, cost 0.904888
spiral i 3, cost 0.762063
spiral i 4, cost 0.461805
spiral i 5, cost 0.000301766
spiral i 6, cost 0.461792
spiral i 7, cost 0.761772
spiral i 8, cost 0.904896
spiral i 9, cost 0.963876
spiral i 10, cost 0.986465
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 208.876, 9.64081, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:08.856246  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 208.876, 9.64081, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963859
spiral i 2, cost 0.904888
spiral i 3, cost 0.762015
spiral i 4, cost 0.461798
spiral i 5, cost 0.000169443
spiral i 6, cost 0.461794
spiral i 7, cost 0.761805
spiral i 8, cost 0.904896
spiral i 9, cost 0.963874
spiral i 10, cost 0.986459
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 210.56, 9.66596, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:09.961187  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 210.56, 9.66596, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986561
spiral i 1, cost 0.963862
spiral i 2, cost 0.904889
spiral i 3, cost 0.761969
spiral i 4, cost 0.461794
spiral i 5, cost 7.24691e-05
spiral i 6, cost 0.461796
spiral i 7, cost 0.761841
spiral i 8, cost 0.904895
spiral i 9, cost 0.963872
spiral i 10, cost 0.986452
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 212.244, 9.69112, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:11.070147  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 212.244, 9.69112, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986436
spiral i 1, cost 0.963865
spiral i 2, cost 0.90489
spiral i 3, cost 0.761931
spiral i 4, cost 0.461791
spiral i 5, cost 2.26938e-05
spiral i 6, cost 0.461797
spiral i 7, cost 0.761874
spiral i 8, cost 0.904894
spiral i 9, cost 0.96387
spiral i 10, cost 0.986446
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 213.928, 9.71627, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:12.137964  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 213.928, 9.71627, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986441
spiral i 1, cost 0.963867
spiral i 2, cost 0.904891
spiral i 3, cost 0.761904
spiral i 4, cost 0.46179
spiral i 5, cost 0.00192026
spiral i 6, cost 0.461798
spiral i 7, cost 0.761901
spiral i 8, cost 0.904893
spiral i 9, cost 0.963868
spiral i 10, cost 0.986441
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 215.612, 9.74143, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:13.222369  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 215.612, 9.74143, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986442
spiral i 1, cost 0.963867
spiral i 2, cost 0.904891
spiral i 3, cost 0.761899
spiral i 4, cost 0.461791
spiral i 5, cost 6.39111e-05
spiral i 6, cost 0.461797
spiral i 7, cost 0.761906
spiral i 8, cost 0.904893
spiral i 9, cost 0.963867
spiral i 10, cost 0.98644
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 217.296, 9.76658, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:14.400113  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 217.296, 9.76658, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986443
spiral i 1, cost 0.963868
spiral i 2, cost 0.904892
spiral i 3, cost 0.761893
spiral i 4, cost 0.461791
spiral i 5, cost 0.00179278
spiral i 6, cost 0.461797
spiral i 7, cost 0.761911
spiral i 8, cost 0.904892
spiral i 9, cost 0.963867
spiral i 10, cost 0.986439
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 218.98, 9.79174, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:15.477947  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 218.98, 9.79174, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986444
spiral i 1, cost 0.963868
spiral i 2, cost 0.904892
spiral i 3, cost 0.761888
spiral i 4, cost 0.461792
spiral i 5, cost 0.00364923
spiral i 6, cost 0.461796
spiral i 7, cost 0.761917
spiral i 8, cost 0.904892
spiral i 9, cost 0.963867
spiral i 10, cost 0.986438
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 220.664, 9.81689, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:16.546641  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
Goal after state_transition
location 220.664, 9.81689, 0
rotation 0.0149366, 0, 0
velocity 2.99967, 0.0448081, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986445
spiral i 1, cost 0.963869
spiral i 2, cost 0.904892
spiral i 3, cost 0.761883
spiral i 4, cost 0.461792
spiral i 5, cost 5.15267e-06
spiral i 6, cost 0.461796
spiral i 7, cost 0.761922
spiral i 8, cost 0.904892
spiral i 9, cost 0.963866
spiral i 10, cost 0.986437
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 222.348, 9.84205, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:17.637218  2964 behavior_planner_FSM.cpp:126] BP- IN FOLLOW_LANE STATE
I0212 14:55:17.637249  2964 behavior_planner_FSM.cpp:129] BP - goal in junction
I0212 14:55:17.637261  2964 behavior_planner_FSM.cpp:132] BP - changing to DECEL_TO_STOP
I0212 14:55:17.637274  2964 behavior_planner_FSM.cpp:135] BP- original STOP goal at: 222.348, 9.84205
I0212 14:55:17.637290  2964 behavior_planner_FSM.cpp:151] BP- new STOP goal at: 221.848, 9.83458
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986563
spiral i 1, cost 0.963844
spiral i 2, cost 0.904891
spiral i 3, cost 0.762062
spiral i 4, cost 0.461807
spiral i 5, cost 4.90693e-06
spiral i 6, cost 0.461811
spiral i 7, cost 0.762108
spiral i 8, cost 0.90489
spiral i 9, cost 0.96384
spiral i 10, cost 0.986563
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 215.927, 9.74614, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:18.807585  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:18.807618  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 5.92106
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986573
spiral i 1, cost 0.963919
spiral i 2, cost 0.904836
spiral i 3, cost 0.761198
spiral i 4, cost 0.461899
spiral i 5, cost 0.00357741
spiral i 6, cost 0.461908
spiral i 7, cost 0.761197
spiral i 8, cost 0.904831
spiral i 9, cost 0.963919
spiral i 10, cost 0.986573
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 217.174, 9.76476, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:19.906440  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:19.906474  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 4.67452
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986584
spiral i 1, cost 0.963941
spiral i 2, cost 0.904343
spiral i 3, cost 0.761189
spiral i 4, cost 0.462111
spiral i 5, cost 0.00357187
spiral i 6, cost 0.462134
spiral i 7, cost 0.761186
spiral i 8, cost 0.904308
spiral i 9, cost 0.963942
spiral i 10, cost 0.986584
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 218.158, 9.77946, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:20.960027  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:20.960093  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 3.6904
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.986592
spiral i 1, cost 0.963965
spiral i 2, cost 0.904972
spiral i 3, cost 0.761025
spiral i 4, cost 0.462597
spiral i 5, cost 0.00356943
spiral i 6, cost 0.462654
spiral i 7, cost 0.761006
spiral i 8, cost 0.904973
spiral i 9, cost 0.963965
spiral i 10, cost 0.986606
Find best spiral, idx 5
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 218.935, 9.79106, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:22.166232  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:22.166280  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 2.91348
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.964012
spiral i 1, cost 0.905019
spiral i 2, cost 0.759698
spiral i 3, cost 0.463689
spiral i 4, cost 0.0035657
spiral i 5, cost 0.463823
spiral i 6, cost 0.759567
spiral i 7, cost 0.90502
Find best spiral, idx 4
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 219.548, 9.80023, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:23.724922  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:23.724954  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 2.30011
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.905098
spiral i 1, cost 0.761342
spiral i 2, cost 0.461737
spiral i 3, cost 0.00356806
spiral i 4, cost 0.46173
spiral i 5, cost 0.761343
spiral i 6, cost 0.905098
Find best spiral, idx 3
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 220.032, 9.80746, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:25.537554  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:25.537590  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 1.81588
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.76141
spiral i 1, cost 0.461565
spiral i 2, cost 0.00356866
spiral i 3, cost 0.461527
spiral i 4, cost 0.761409
Find best spiral, idx 2
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 220.414, 9.81317, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:26.870152  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:26.870182  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 1.4336
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.460198
spiral i 1, cost 0.00356865
spiral i 2, cost 0.459945
Find best spiral, idx 1
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 220.716, 9.81768, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:27.972350  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:27.972384  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 1.13179
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.461885
spiral i 1, cost 0.00356961
spiral i 2, cost 0.461886
Find best spiral, idx 1
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 220.954, 9.82123, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:28.746237  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:28.746271  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 0.893519
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.461946
spiral i 1, cost 0.00356427
spiral i 2, cost 0.461945
Find best spiral, idx 1
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 221.143, 9.82404, 0
rotation -6.26825, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:29.408849  2964 behavior_planner_FSM.cpp:178] BP- IN DECEL_TO_STOP STATE
I0212 14:55:29.408886  2964 behavior_planner_FSM.cpp:196] Ego distance to stop line: 0.705414
I0212 14:55:29.408922  2964 behavior_planner_FSM.cpp:208] BP - changing to STOPPED
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
spiral i 0, cost 0.00356303
spiral i 1, cost 0.761247
Find best spiral, idx 0
behavior_and_motion_planner finish

behavior_and_motion_planner start
Goal before state_transition
location 221.291, 9.82626, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:29.836438  2964 behavior_planner_FSM.cpp:212] BP- IN STOPPED STATE
I0212 14:55:29.836488  2964 behavior_planner_FSM.cpp:224] BP- Stopped for 0 secs
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0

behavior_and_motion_planner start
Goal before state_transition
location 221.291, 9.82626, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:30.237586  2964 behavior_planner_FSM.cpp:212] BP- IN STOPPED STATE
I0212 14:55:30.237622  2964 behavior_planner_FSM.cpp:224] BP- Stopped for 0 secs
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0

behavior_and_motion_planner start
Goal before state_transition
location 221.291, 9.82626, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
I0212 14:55:30.633777  2964 behavior_planner_FSM.cpp:212] BP- IN STOPPED STATE
I0212 14:55:30.633823  2964 behavior_planner_FSM.cpp:224] BP- Stopped for 1 secs
I0212 14:55:30.634029  2964 behavior_planner_FSM.cpp:231] BP - changing to FOLLOW_LANE
Goal after state_transition
location 221.848, 9.83458, 0
rotation 0.0149366, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
key board interrupt, good bye
*/