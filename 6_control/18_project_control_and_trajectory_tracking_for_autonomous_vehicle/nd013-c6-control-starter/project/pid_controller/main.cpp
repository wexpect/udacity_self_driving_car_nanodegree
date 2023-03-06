/**********************************************
 
 Git Repo:
 https://github.com/udacity/nd013-c6-control-starter

 
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
#include <iostream>
#include <fstream>
#include <typeinfo>

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
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

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


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  std::cout << "\npath_planner start" << std::endl;

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];


    std::cout<< "points" << std::endl;
    std::cout << x_points[x_points.size()-2] << ", " << y_points[y_points.size()-2] << std::endl;
    std::cout << x_points[x_points.size()-1] << ", " << y_points[y_points.size()-1] << std::endl;
    std::cout<< "ego_state.rotation.yaw " << ego_state.rotation.yaw << std::endl;

    std::cout<< "yaw " << yaw << std::endl;    

  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;

  }

  Maneuver behavior = behavior_planner.get_active_maneuver();
  std::cout << "maneuver before: " << behavior << std::endl;

  std::cout << "Goal before state_transition" << std::endl;
  std::cout << "location " << goal.location.x << ", " << goal.location.y << ", " << goal.location.z << std::endl;
  std::cout << "rotation " << goal.rotation.yaw << ", " << goal.rotation.pitch << ", " << goal.rotation.roll << std::endl;
  std::cout << "velocity " << goal.velocity.x << ", " << goal.velocity.y << ", " << goal.velocity.z << std::endl;
  std::cout << "acceleration " << goal.acceleration.x << ", " << goal.acceleration.y << ", " << goal.acceleration.z << std::endl;

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  std::cout << "maneuver after: " << behavior << std::endl;

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

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

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

  if(best_spirals.size() > 0){
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

  std::cout << "path_planner finish" << std::endl;
}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);



  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/
  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/

  PID pid_steer = PID();
  pid_steer.Init(0.005, 0.0005, 0.1, 1.2, -1.2);

  PID pid_throttle = PID();
  pid_throttle.Init(0.03, 0.004, 0.2, 1, -1);


  // PID pid_steer = PID();
  // // pid_steer.Init(0.5, 0.0005, 0.5, 1.2, -1.2);
  // pid_steer.Init(1.0, 0.0005, 10.0, 1.2, -1.2);  

  // PID pid_throttle = PID();
  // pid_throttle.Init(0.1, 0, 0.02, 1, -1);



  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {
          
          std::cout<< "i " << i << std::endl;

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");


          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];  // NOTE: ego yaw
          double velocity = data["velocity"];  // NOTE: ego velocity
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          // NOTE: ego position
          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

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


          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);


          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          std::cout << "timer " << timer << ", prev_timer " << prev_timer << ", new_delta_time " << new_delta_time << std::endl;   
          prev_timer = timer;


          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          double error_steer;
          double steer_output;

          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
          **/
          std::cout << "x_position " << x_position << ", x_desired " << x_points[x_points.size() - 1] << std::endl;
          std::cout << "y_position " << y_position << ", y_desired " << y_points[y_points.size() - 1] << std::endl;
          error_steer = y_position - y_points[y_points.size() - 1];


          // std::cout << "x_position " << x_position << ", x_desired " << x_points[x_points.size() - 1] << std::endl;
          // std::cout << "y_position " << y_position << ", y_desired " << y_points[y_points.size() - 1] << std::endl;
          // double theta_real = yaw;
          // double theta_desired = angle_between_points(x_position, y_position, x_points[x_points.size() - 1], y_points[y_points.size() - 1]);
          // std::cout << "yaw " << yaw << ", theta_real " << theta_real << ", theta_desired " << theta_desired << std::endl;
          // error_steer = theta_real - theta_desired;




          /**
          * TODO (step 3): uncomment these lines
          **/
          // Compute control to apply
          pid_steer.UpdateError(error_steer);
          steer_output = pid_steer.TotalError();

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;




          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines
          **/
          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;


          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          error_throttle = velocity - v_points[v_points.size() - 1];
          std::cout << "velocity " << velocity << ", velocity_goal " << v_points[v_points.size() - 1] << std::endl;
             

          double throttle_output;
          double brake_output;

          /**
          * TODO (step 2): uncomment these lines
          **/
          // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          double throttle = pid_throttle.TotalError();

          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;



          // Send control
          json msgJson;

          // NOTE: here assign throttle and steer in Control.
          // see simulatorAPI.py L312
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;    // for plotting
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();          

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
path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9401, y_desired 15.8693
err -4.92913, sum_err -1700.17, diff_err 0, -kp*err 0.0246457, -ki*sum_err 0.850085, -kd*diff_err -0, control 0.874731, control_lim 0.874731
velocity 0.0445451, velocity_goal 3
err -2.95545, sum_err -1139.64, diff_err 0, -kp*err 0.0886636, -ki*sum_err 4.55855, -kd*diff_err -0, control 4.64722, control_lim 1
steer:  0.874730941826156
throttle:  1.0
brake:  0.0
velocity sent:  0.04155632396678542
i 453

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.944, y_desired 15.8693
err -4.92522, sum_err -1705.1, diff_err 0.00391293, -kp*err 0.0246261, -ki*sum_err 0.852548, -kd*diff_err -0.000391293, control 0.876783, control_lim 0.876783
velocity 0.0415563, velocity_goal 3
err -2.95844, sum_err -1142.6, diff_err -0.00298876, -kp*err 0.0887533, -ki*sum_err 4.57039, -kd*diff_err 0.000597752, control 4.65974, control_lim 1
steer:  0.876782694870284
throttle:  1.0
brake:  0.0
velocity sent:  0.04155632396678542
i 454

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.944, y_desired 15.8693
err -4.92522, sum_err -1710.02, diff_err 0, -kp*err 0.0246261, -ki*sum_err 0.85501, -kd*diff_err -0, control 0.879637, control_lim 0.879637
velocity 0.0415563, velocity_goal 3
err -2.95844, sum_err -1145.55, diff_err 0, -kp*err 0.0887533, -ki*sum_err 4.58222, -kd*diff_err -0, control 4.67097, control_lim 1
steer:  0.879636597687057
throttle:  1.0
brake:  0.0
velocity sent:  0.04155632396678542
i 455

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.944, y_desired 15.8693
err -4.92522, sum_err -1714.95, diff_err 0, -kp*err 0.0246261, -ki*sum_err 0.857473, -kd*diff_err -0, control 0.882099, control_lim 0.882099
velocity 0.0415563, velocity_goal 3
err -2.95844, sum_err -1148.51, diff_err 0, -kp*err 0.0887533, -ki*sum_err 4.59405, -kd*diff_err -0, control 4.68281, control_lim 1
steer:  0.882099207931808
throttle:  1.0
brake:  0.0
velocity sent:  0.04155632396678542
i 456

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.944, y_desired 15.8693
err -4.92522, sum_err -1719.87, diff_err 0, -kp*err 0.0246261, -ki*sum_err 0.859936, -kd*diff_err -0, control 0.884562, control_lim 0.884562
velocity 0.0415563, velocity_goal 3
err -2.95844, sum_err -1151.47, diff_err 0, -kp*err 0.0887533, -ki*sum_err 4.60589, -kd*diff_err -0, control 4.69464, control_lim 1
steer:  0.884561818176559
throttle:  1.0
brake:  0.0
velocity sent:  0.0436259546554077
i 457

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9481, y_desired 15.8693
err -4.92116, sum_err -1724.79, diff_err 0.00405598, -kp*err 0.0246058, -ki*sum_err 0.862396, -kd*diff_err -0.000405598, control 0.886597, control_lim 0.886597
velocity 0.043626, velocity_goal 3
err -2.95637, sum_err -1154.43, diff_err 0.00206963, -kp*err 0.0886912, -ki*sum_err 4.61771, -kd*diff_err -0.000413926, control 4.70599, control_lim 1
steer:  0.88659652286177
throttle:  1.0
brake:  0.0
velocity sent:  0.0436259546554077
i 458

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9481, y_desired 15.8693
err -4.92116, sum_err -1729.71, diff_err 0, -kp*err 0.0246058, -ki*sum_err 0.864857, -kd*diff_err -0, control 0.889463, control_lim 0.889463
velocity 0.043626, velocity_goal 3
err -2.95637, sum_err -1157.38, diff_err 0, -kp*err 0.0886912, -ki*sum_err 4.62954, -kd*diff_err -0, control 4.71823, control_lim 1
steer:  0.889462702804855
throttle:  1.0
brake:  0.0
velocity sent:  0.0436259546554077
i 459

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9481, y_desired 15.8693
err -4.92116, sum_err -1734.63, diff_err 0, -kp*err 0.0246058, -ki*sum_err 0.867317, -kd*diff_err -0, control 0.891923, control_lim 0.891923
velocity 0.043626, velocity_goal 3
err -2.95637, sum_err -1160.34, diff_err 0, -kp*err 0.0886912, -ki*sum_err 4.64136, -kd*diff_err -0, control 4.73005, control_lim 1
steer:  0.891923285061172
throttle:  1.0
brake:  0.0
velocity sent:  0.0436259546554077
i 460

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9481, y_desired 15.8693
err -4.92116, sum_err -1739.56, diff_err 0, -kp*err 0.0246058, -ki*sum_err 0.869778, -kd*diff_err -0, control 0.894384, control_lim 0.894384
velocity 0.043626, velocity_goal 3
err -2.95637, sum_err -1163.3, diff_err 0, -kp*err 0.0886912, -ki*sum_err 4.65319, -kd*diff_err -0, control 4.74188, control_lim 1
steer:  0.894383867317489
throttle:  1.0
brake:  0.0
velocity sent:  0.04822577424577086
i 461

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9528, y_desired 15.8693
err -4.91644, sum_err -1744.47, diff_err 0.00472832, -kp*err 0.0245822, -ki*sum_err 0.872236, -kd*diff_err -0.000472832, control 0.896346, control_lim 0.896346
velocity 0.0482258, velocity_goal 3
err -2.95177, sum_err -1166.25, diff_err 0.00459982, -kp*err 0.0885532, -ki*sum_err 4.665, -kd*diff_err -0.000919964, control 4.75263, control_lim 1
steer:  0.896345612102798
throttle:  1.0
brake:  0.0
velocity sent:  0.04822577424577086
i 462

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9528, y_desired 15.8693
err -4.91644, sum_err -1749.39, diff_err 0, -kp*err 0.0245822, -ki*sum_err 0.874694, -kd*diff_err -0, control 0.899277, control_lim 0.899277
velocity 0.0482258, velocity_goal 3
err -2.95177, sum_err -1169.2, diff_err 0, -kp*err 0.0885532, -ki*sum_err 4.6768, -kd*diff_err -0, control 4.76536, control_lim 1
steer:  0.899276661926559
throttle:  1.0
brake:  0.0
velocity sent:  0.04822577424577086
i 463

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9528, y_desired 15.8693
err -4.91644, sum_err -1754.31, diff_err 0, -kp*err 0.0245822, -ki*sum_err 0.877153, -kd*diff_err -0, control 0.901735, control_lim 0.901735
velocity 0.0482258, velocity_goal 3
err -2.95177, sum_err -1172.15, diff_err 0, -kp*err 0.0885532, -ki*sum_err 4.68861, -kd*diff_err -0, control 4.77716, control_lim 1
steer:  0.901734880024246
throttle:  1.0
brake:  0.0
velocity sent:  0.04822577424577086
i 464

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9528, y_desired 15.8693
err -4.91644, sum_err -1759.22, diff_err 0, -kp*err 0.0245822, -ki*sum_err 0.879611, -kd*diff_err -0, control 0.904193, control_lim 0.904193
velocity 0.0482258, velocity_goal 3
err -2.95177, sum_err -1175.1, diff_err 0, -kp*err 0.0885532, -ki*sum_err 4.70042, -kd*diff_err -0, control 4.78897, control_lim 1
steer:  0.904193098121932
throttle:  1.0
brake:  0.0
velocity sent:  0.047097698986900936
i 465

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9572, y_desired 15.8693
err -4.91203, sum_err -1764.13, diff_err 0.00440884, -kp*err 0.0245601, -ki*sum_err 0.882067, -kd*diff_err -0.000440884, control 0.906186, control_lim 0.906186
velocity 0.0470977, velocity_goal 3
err -2.9529, sum_err -1178.06, diff_err -0.00112808, -kp*err 0.0885871, -ki*sum_err 4.71223, -kd*diff_err 0.000225615, control 4.80104, control_lim 1
steer:  0.906186183983138
throttle:  1.0
brake:  0.0
velocity sent:  0.047097698986900936
i 466

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9572, y_desired 15.8693
err -4.91203, sum_err -1769.05, diff_err 0, -kp*err 0.0245601, -ki*sum_err 0.884523, -kd*diff_err -0, control 0.909083, control_lim 0.909083
velocity 0.0470977, velocity_goal 3
err -2.9529, sum_err -1181.01, diff_err 0, -kp*err 0.0885871, -ki*sum_err 4.72404, -kd*diff_err -0, control 4.81263, control_lim 1
steer:  0.909083081299117
throttle:  1.0
brake:  0.0
velocity sent:  0.047097698986900936
i 467

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9572, y_desired 15.8693
err -4.91203, sum_err -1773.96, diff_err 0, -kp*err 0.0245601, -ki*sum_err 0.886979, -kd*diff_err -0, control 0.911539, control_lim 0.911539
velocity 0.0470977, velocity_goal 3
err -2.9529, sum_err -1183.96, diff_err 0, -kp*err 0.0885871, -ki*sum_err 4.73585, -kd*diff_err -0, control 4.82444, control_lim 1
steer:  0.911539094978622
throttle:  1.0
brake:  0.0
velocity sent:  0.047097698986900936
i 468

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9572, y_desired 15.8693
err -4.91203, sum_err -1778.87, diff_err 0, -kp*err 0.0245601, -ki*sum_err 0.889435, -kd*diff_err -0, control 0.913995, control_lim 0.913995
velocity 0.0470977, velocity_goal 3
err -2.9529, sum_err -1186.92, diff_err 0, -kp*err 0.0885871, -ki*sum_err 4.74766, -kd*diff_err -0, control 4.83625, control_lim 1
steer:  0.913995108658126
throttle:  1.0
brake:  0.0
velocity sent:  0.04473585807093578
i 469

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9612, y_desired 15.8693
err -4.90804, sum_err -1783.78, diff_err 0.00398445, -kp*err 0.0245402, -ki*sum_err 0.891889, -kd*diff_err -0.000398445, control 0.916031, control_lim 0.916031
velocity 0.0447359, velocity_goal 3
err -2.95526, sum_err -1189.87, diff_err -0.00236184, -kp*err 0.0886579, -ki*sum_err 4.75948, -kd*diff_err 0.000472368, control 4.84861, control_lim 1
steer:  0.916030762726119
throttle:  1.0
brake:  0.0
velocity sent:  0.04473585807093578
i 470

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9612, y_desired 15.8693
err -4.90804, sum_err -1788.69, diff_err 0, -kp*err 0.0245402, -ki*sum_err 0.894343, -kd*diff_err -0, control 0.918883, control_lim 0.918883
velocity 0.0447359, velocity_goal 3
err -2.95526, sum_err -1192.83, diff_err 0, -kp*err 0.0886579, -ki*sum_err 4.77131, -kd*diff_err -0, control 4.85996, control_lim 1
steer:  0.918883229309371
throttle:  1.0
brake:  0.0
velocity sent:  0.04473585807093578
i 471

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9612, y_desired 15.8693
err -4.90804, sum_err -1793.59, diff_err 0, -kp*err 0.0245402, -ki*sum_err 0.896797, -kd*diff_err -0, control 0.921337, control_lim 0.921337
velocity 0.0447359, velocity_goal 3
err -2.95526, sum_err -1195.78, diff_err 0, -kp*err 0.0886579, -ki*sum_err 4.78313, -kd*diff_err -0, control 4.87178, control_lim 1
steer:  0.921337250763229
throttle:  1.0
brake:  0.0
velocity sent:  0.04473585807093578
i 472

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9612, y_desired 15.8693
err -4.90804, sum_err -1798.5, diff_err 0, -kp*err 0.0245402, -ki*sum_err 0.899251, -kd*diff_err -0, control 0.923791, control_lim 0.923791
velocity 0.0447359, velocity_goal 3
err -2.95526, sum_err -1198.74, diff_err 0, -kp*err 0.0886579, -ki*sum_err 4.79495, -kd*diff_err -0, control 4.88361, control_lim 1
steer:  0.923791272217086
throttle:  1.0
brake:  0.0
velocity sent:  0.046268240476280593
i 473

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9654, y_desired 15.8693
err -4.90388, sum_err -1803.41, diff_err 0.00415993, -kp*err 0.0245194, -ki*sum_err 0.901703, -kd*diff_err -0.000415993, control 0.925806, control_lim 0.925806
velocity 0.0462682, velocity_goal 3
err -2.95373, sum_err -1201.69, diff_err 0.00153238, -kp*err 0.088612, -ki*sum_err 4.80676, -kd*diff_err -0.000306476, control 4.89507, control_lim 1
steer:  0.925806421333602
throttle:  1.0
brake:  0.0
velocity sent:  0.046268240476280593
i 474

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9654, y_desired 15.8693
err -4.90388, sum_err -1808.31, diff_err 0, -kp*err 0.0245194, -ki*sum_err 0.904155, -kd*diff_err -0, control 0.928674, control_lim 0.928674
velocity 0.0462682, velocity_goal 3
err -2.95373, sum_err -1204.64, diff_err 0, -kp*err 0.088612, -ki*sum_err 4.81858, -kd*diff_err -0, control 4.90719, control_lim 1
steer:  0.928674355560592
throttle:  1.0
brake:  0.0
velocity sent:  0.046268240476280593
i 475

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029696, prev_timer 1678029696, new_delta_time 0
y_position 10.9654, y_desired 15.8693
err -4.90388, sum_err -1813.21, diff_err 0, -kp*err 0.0245194, -ki*sum_err 0.906607, -kd*diff_err -0, control 0.931126, control_lim 0.931126
velocity 0.0462682, velocity_goal 3
err -2.95373, sum_err -1207.6, diff_err 0, -kp*err 0.088612, -ki*sum_err 4.83039, -kd*diff_err -0, control 4.919, control_lim 1
steer:  0.931126297050765
throttle:  1.0
brake:  0.0
velocity sent:  0.046268240476280593
i 476

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029696, new_delta_time 1
y_position 10.9654, y_desired 15.8693
err -4.90388, sum_err -1818.12, diff_err 0, -kp*err 0.0245194, -ki*sum_err 0.909059, -kd*diff_err -0, control 0.933578, control_lim 0.933578
velocity 0.0462682, velocity_goal 3
err -2.95373, sum_err -1210.55, diff_err 0, -kp*err 0.088612, -ki*sum_err 4.84221, -kd*diff_err -0, control 4.93082, control_lim 1
steer:  0.933578238540939
throttle:  1.0
brake:  0.0
velocity sent:  0.046399729648304675
i 477

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9695, y_desired 15.8693
err -4.89977, sum_err -1823.02, diff_err 0.00411415, -kp*err 0.0244988, -ki*sum_err 0.911509, -kd*diff_err -0.000411415, control 0.935596, control_lim 0.935596
velocity 0.0463997, velocity_goal 3
err -2.9536, sum_err -1213.51, diff_err 0.000131489, -kp*err 0.088608, -ki*sum_err 4.85402, -kd*diff_err -2.62978e-05, control 4.9426, control_lim 1
steer:  0.935596137100509
throttle:  1.0
brake:  0.0
velocity sent:  0.046399729648304675
i 478

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9695, y_desired 15.8693
err -4.89977, sum_err -1827.92, diff_err 0, -kp*err 0.0244988, -ki*sum_err 0.913959, -kd*diff_err -0, control 0.938457, control_lim 0.938457
velocity 0.0463997, velocity_goal 3
err -2.9536, sum_err -1216.46, diff_err 0, -kp*err 0.088608, -ki*sum_err 4.86584, -kd*diff_err -0, control 4.95444, control_lim 1
steer:  0.938457436615279
throttle:  1.0
brake:  0.0
velocity sent:  0.046399729648304675
i 479

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9695, y_desired 15.8693
err -4.89977, sum_err -1832.82, diff_err 0, -kp*err 0.0244988, -ki*sum_err 0.916408, -kd*diff_err -0, control 0.940907, control_lim 0.940907
velocity 0.0463997, velocity_goal 3
err -2.9536, sum_err -1219.41, diff_err 0, -kp*err 0.088608, -ki*sum_err 4.87765, -kd*diff_err -0, control 4.96626, control_lim 1
steer:  0.940907321029952
throttle:  1.0
brake:  0.0
velocity sent:  0.046399729648304675
i 480

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9695, y_desired 15.8693
err -4.89977, sum_err -1837.72, diff_err 0, -kp*err 0.0244988, -ki*sum_err 0.918858, -kd*diff_err -0, control 0.943357, control_lim 0.943357
velocity 0.0463997, velocity_goal 3
err -2.9536, sum_err -1222.37, diff_err 0, -kp*err 0.088608, -ki*sum_err 4.88947, -kd*diff_err -0, control 4.97807, control_lim 1
steer:  0.943357205444625
throttle:  1.0
brake:  0.0
velocity sent:  0.046399729648304675
i 481

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9695, y_desired 15.8693
err -4.89977, sum_err -1842.62, diff_err 0, -kp*err 0.0244988, -ki*sum_err 0.921308, -kd*diff_err -0, control 0.945807, control_lim 0.945807
velocity 0.0463997, velocity_goal 3
err -2.9536, sum_err -1225.32, diff_err 0, -kp*err 0.088608, -ki*sum_err 4.90128, -kd*diff_err -0, control 4.98989, control_lim 1
steer:  0.945807089859298
throttle:  1.0
brake:  0.0
velocity sent:  0.04946444740854188
i 482

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9733, y_desired 15.8693
err -4.89597, sum_err -1847.51, diff_err 0.00379467, -kp*err 0.0244799, -ki*sum_err 0.923756, -kd*diff_err -0.000379467, control 0.947857, control_lim 0.947857
velocity 0.0494644, velocity_goal 3
err -2.95054, sum_err -1228.27, diff_err 0.00306472, -kp*err 0.0885161, -ki*sum_err 4.91308, -kd*diff_err -0.000612944, control 5.00098, control_lim 1
steer:  0.947856636577895
throttle:  1.0
brake:  0.0
velocity sent:  0.04946444740854188
i 483

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9733, y_desired 15.8693
err -4.89597, sum_err -1852.41, diff_err 0, -kp*err 0.0244799, -ki*sum_err 0.926204, -kd*diff_err -0, control 0.950684, control_lim 0.950684
velocity 0.0494644, velocity_goal 3
err -2.95054, sum_err -1231.22, diff_err 0, -kp*err 0.0885161, -ki*sum_err 4.92488, -kd*diff_err -0, control 5.0134, control_lim 1
steer:  0.950684090668014
throttle:  1.0
brake:  0.0
velocity sent:  0.04946444740854188
i 484

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9733, y_desired 15.8693
err -4.89597, sum_err -1857.3, diff_err 0, -kp*err 0.0244799, -ki*sum_err 0.928652, -kd*diff_err -0, control 0.953132, control_lim 0.953132
velocity 0.0494644, velocity_goal 3
err -2.95054, sum_err -1234.17, diff_err 0, -kp*err 0.0885161, -ki*sum_err 4.93669, -kd*diff_err -0, control 5.0252, control_lim 1
steer:  0.953132077747634
throttle:  1.0
brake:  0.0
velocity sent:  0.04946444740854188
i 485

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9733, y_desired 15.8693
err -4.89597, sum_err -1862.2, diff_err 0, -kp*err 0.0244799, -ki*sum_err 0.9311, -kd*diff_err -0, control 0.95558, control_lim 0.95558
velocity 0.0494644, velocity_goal 3
err -2.95054, sum_err -1237.12, diff_err 0, -kp*err 0.0885161, -ki*sum_err 4.94849, -kd*diff_err -0, control 5.037, control_lim 1
steer:  0.955580064827255
throttle:  1.0
brake:  0.0
velocity sent:  0.049472645385159546
i 486

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.978, y_desired 15.8693
err -4.89123, sum_err -1867.09, diff_err 0.00474262, -kp*err 0.0244562, -ki*sum_err 0.933546, -kd*diff_err -0.000474262, control 0.957528, control_lim 0.957528
velocity 0.0494726, velocity_goal 3
err -2.95053, sum_err -1240.07, diff_err 8.19798e-06, -kp*err 0.0885158, -ki*sum_err 4.96029, -kd*diff_err -1.6396e-06, control 5.0488, control_lim 1
steer:  0.957527705246261
throttle:  1.0
brake:  0.0
velocity sent:  0.049472645385159546
i 487

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.978, y_desired 15.8693
err -4.89123, sum_err -1871.98, diff_err 0, -kp*err 0.0244562, -ki*sum_err 0.935991, -kd*diff_err -0, control 0.960448, control_lim 0.960448
velocity 0.0494726, velocity_goal 3
err -2.95053, sum_err -1243.02, diff_err 0, -kp*err 0.0885158, -ki*sum_err 4.97209, -kd*diff_err -0, control 5.06061, control_lim 1
steer:  0.960447583252242
throttle:  1.0
brake:  0.0
velocity sent:  0.049472645385159546
i 488

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.978, y_desired 15.8693
err -4.89123, sum_err -1876.87, diff_err 0, -kp*err 0.0244562, -ki*sum_err 0.938437, -kd*diff_err -0, control 0.962893, control_lim 0.962893
velocity 0.0494726, velocity_goal 3
err -2.95053, sum_err -1245.97, diff_err 0, -kp*err 0.0885158, -ki*sum_err 4.98389, -kd*diff_err -0, control 5.07241, control_lim 1
steer:  0.962893199020675
throttle:  1.0
brake:  0.0
velocity sent:  0.049472645385159546
i 489

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.978, y_desired 15.8693
err -4.89123, sum_err -1881.77, diff_err 0, -kp*err 0.0244562, -ki*sum_err 0.940883, -kd*diff_err -0, control 0.965339, control_lim 0.965339
velocity 0.0494726, velocity_goal 3
err -2.95053, sum_err -1248.92, diff_err 0, -kp*err 0.0885158, -ki*sum_err 4.9957, -kd*diff_err -0, control 5.08421, control_lim 1
steer:  0.965338814789108
throttle:  1.0
brake:  0.0
velocity sent:  0.0478230963141198
i 490

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9816, y_desired 15.8693
err -4.88765, sum_err -1886.65, diff_err 0.00357819, -kp*err 0.0244383, -ki*sum_err 0.943326, -kd*diff_err -0.000357819, control 0.967407, control_lim 0.967407
velocity 0.0478231, velocity_goal 3
err -2.95218, sum_err -1251.88, diff_err -0.00164955, -kp*err 0.0885653, -ki*sum_err 5.00751, -kd*diff_err 0.00032991, control 5.0964, control_lim 1
steer:  0.967406931930831
throttle:  1.0
brake:  0.0
velocity sent:  0.0478230963141198
i 491

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9816, y_desired 15.8693
err -4.88765, sum_err -1891.54, diff_err 0, -kp*err 0.0244383, -ki*sum_err 0.94577, -kd*diff_err -0, control 0.970209, control_lim 0.970209
velocity 0.0478231, velocity_goal 3
err -2.95218, sum_err -1254.83, diff_err 0, -kp*err 0.0885653, -ki*sum_err 5.01931, -kd*diff_err -0, control 5.10788, control_lim 1
steer:  0.970208577209762
throttle:  1.0
brake:  0.0
velocity sent:  0.0478230963141198
i 492

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9816, y_desired 15.8693
err -4.88765, sum_err -1896.43, diff_err 0, -kp*err 0.0244383, -ki*sum_err 0.948214, -kd*diff_err -0, control 0.972652, control_lim 0.972652
velocity 0.0478231, velocity_goal 3
err -2.95218, sum_err -1257.78, diff_err 0, -kp*err 0.0885653, -ki*sum_err 5.03112, -kd*diff_err -0, control 5.11969, control_lim 1
steer:  0.972652403885177
throttle:  1.0
brake:  0.0
velocity sent:  0.0478230963141198
i 493

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9816, y_desired 15.8693
err -4.88765, sum_err -1901.32, diff_err 0, -kp*err 0.0244383, -ki*sum_err 0.950658, -kd*diff_err -0, control 0.975096, control_lim 0.975096
velocity 0.0478231, velocity_goal 3
err -2.95218, sum_err -1260.73, diff_err 0, -kp*err 0.0885653, -ki*sum_err 5.04293, -kd*diff_err -0, control 5.1315, control_lim 1
steer:  0.975096230560592
throttle:  1.0
brake:  0.0
velocity sent:  0.05004478622294278
i 494

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9865, y_desired 15.8693
err -4.88277, sum_err -1906.2, diff_err 0.00488091, -kp*err 0.0244139, -ki*sum_err 0.953099, -kd*diff_err -0.000488091, control 0.977025, control_lim 0.977025
velocity 0.0500448, velocity_goal 3
err -2.94996, sum_err -1263.68, diff_err 0.00222169, -kp*err 0.0884987, -ki*sum_err 5.05473, -kd*diff_err -0.000444338, control 5.14279, control_lim 1
steer:  0.977025121742538
throttle:  1.0
brake:  0.0
velocity sent:  0.05004478622294278
i 495

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9865, y_desired 15.8693
err -4.88277, sum_err -1911.08, diff_err 0, -kp*err 0.0244139, -ki*sum_err 0.955541, -kd*diff_err -0, control 0.979955, control_lim 0.979955
velocity 0.0500448, velocity_goal 3
err -2.94996, sum_err -1266.63, diff_err 0, -kp*err 0.0884987, -ki*sum_err 5.06653, -kd*diff_err -0, control 5.15503, control_lim 1
steer:  0.979954598480514
throttle:  1.0
brake:  0.0
velocity sent:  0.05004478622294278
i 496

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9865, y_desired 15.8693
err -4.88277, sum_err -1915.96, diff_err 0, -kp*err 0.0244139, -ki*sum_err 0.957982, -kd*diff_err -0, control 0.982396, control_lim 0.982396
velocity 0.0500448, velocity_goal 3
err -2.94996, sum_err -1269.58, diff_err 0, -kp*err 0.0884987, -ki*sum_err 5.07833, -kd*diff_err -0, control 5.16683, control_lim 1
steer:  0.982395984703353
throttle:  1.0
brake:  0.0
velocity sent:  0.05004478622294278
i 497

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9865, y_desired 15.8693
err -4.88277, sum_err -1920.85, diff_err 0, -kp*err 0.0244139, -ki*sum_err 0.960424, -kd*diff_err -0, control 0.984837, control_lim 0.984837
velocity 0.0500448, velocity_goal 3
err -2.94996, sum_err -1272.53, diff_err 0, -kp*err 0.0884987, -ki*sum_err 5.09013, -kd*diff_err -0, control 5.17863, control_lim 1
steer:  0.984837370926193
throttle:  1.0
brake:  0.0
velocity sent:  0.04756137887270046
i 498

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9909, y_desired 15.8693
err -4.87836, sum_err -1925.73, diff_err 0.00440884, -kp*err 0.0243918, -ki*sum_err 0.962863, -kd*diff_err -0.000440884, control 0.986814, control_lim 0.986814
velocity 0.0475614, velocity_goal 3
err -2.95244, sum_err -1275.49, diff_err -0.00248341, -kp*err 0.0885732, -ki*sum_err 5.10194, -kd*diff_err 0.000496681, control 5.19101, control_lim 1
steer:  0.986813624912551
throttle:  1.0
brake:  0.0
velocity sent:  0.04756137887270046
i 499

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9909, y_desired 15.8693
err -4.87836, sum_err -1930.6, diff_err 0, -kp*err 0.0243918, -ki*sum_err 0.965302, -kd*diff_err -0, control 0.989694, control_lim 0.989694
velocity 0.0475614, velocity_goal 3
err -2.95244, sum_err -1278.44, diff_err 0, -kp*err 0.0885732, -ki*sum_err 5.11375, -kd*diff_err -0, control 5.20232, control_lim 1
steer:  0.989693690353683
throttle:  1.0
brake:  0.0
velocity sent:  0.04756137887270046
i 500

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9909, y_desired 15.8693
err -4.87836, sum_err -1935.48, diff_err 0, -kp*err 0.0243918, -ki*sum_err 0.967741, -kd*diff_err -0, control 0.992133, control_lim 0.992133
velocity 0.0475614, velocity_goal 3
err -2.95244, sum_err -1281.39, diff_err 0, -kp*err 0.0885732, -ki*sum_err 5.12556, -kd*diff_err -0, control 5.21413, control_lim 1
steer:  0.99213287215834
throttle:  1.0
brake:  0.0
velocity sent:  0.04756137887270046
i 501

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9909, y_desired 15.8693
err -4.87836, sum_err -1940.36, diff_err 0, -kp*err 0.0243918, -ki*sum_err 0.97018, -kd*diff_err -0, control 0.994572, control_lim 0.994572
velocity 0.0475614, velocity_goal 3
err -2.95244, sum_err -1284.34, diff_err 0, -kp*err 0.0885732, -ki*sum_err 5.13737, -kd*diff_err -0, control 5.22594, control_lim 1
steer:  0.994572053962997
throttle:  1.0
brake:  0.0
velocity sent:  0.04641063053263148
i 502

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9949, y_desired 15.8693
err -4.8743, sum_err -1945.23, diff_err 0.00406075, -kp*err 0.0243715, -ki*sum_err 0.972617, -kd*diff_err -0.000406075, control 0.996583, control_lim 0.996583
velocity 0.0464106, velocity_goal 3
err -2.95359, sum_err -1287.3, diff_err -0.00115075, -kp*err 0.0886077, -ki*sum_err 5.14918, -kd*diff_err 0.00023015, control 5.23802, control_lim 1
steer:  0.996582827144912
throttle:  1.0
brake:  0.0
velocity sent:  0.04641063053263148
i 503

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9949, y_desired 15.8693
err -4.8743, sum_err -1950.11, diff_err 0, -kp*err 0.0243715, -ki*sum_err 0.975055, -kd*diff_err -0, control 0.999426, control_lim 0.999426
velocity 0.0464106, velocity_goal 3
err -2.95359, sum_err -1290.25, diff_err 0, -kp*err 0.0886077, -ki*sum_err 5.161, -kd*diff_err -0, control 5.24961, control_lim 1
steer:  0.999426053100875
throttle:  1.0
brake:  0.0
velocity sent:  0.04641063053263148
i 504

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9949, y_desired 15.8693
err -4.8743, sum_err -1954.98, diff_err 0, -kp*err 0.0243715, -ki*sum_err 0.977492, -kd*diff_err -0, control 1.00186, control_lim 1.00186
velocity 0.0464106, velocity_goal 3
err -2.95359, sum_err -1293.2, diff_err 0, -kp*err 0.0886077, -ki*sum_err 5.17281, -kd*diff_err -0, control 5.26142, control_lim 1
steer:  1.00186320453291
throttle:  1.0
brake:  0.0
velocity sent:  0.04641063053263148
i 505

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9949, y_desired 15.8693
err -4.8743, sum_err -1959.86, diff_err 0, -kp*err 0.0243715, -ki*sum_err 0.979929, -kd*diff_err -0, control 1.0043, control_lim 1.0043
velocity 0.0464106, velocity_goal 3
err -2.95359, sum_err -1296.16, diff_err 0, -kp*err 0.0886077, -ki*sum_err 5.18463, -kd*diff_err -0, control 5.27323, control_lim 1
steer:  1.00430035596495
throttle:  1.0
brake:  0.0
velocity sent:  0.036941951259807676
i 506

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9992, y_desired 15.8693
err -4.87005, sum_err -1964.73, diff_err 0.00424862, -kp*err 0.0243503, -ki*sum_err 0.982364, -kd*diff_err -0.000424862, control 1.00629, control_lim 1.00629
velocity 0.036942, velocity_goal 3
err -2.96306, sum_err -1299.12, diff_err -0.00946868, -kp*err 0.0888917, -ki*sum_err 5.19648, -kd*diff_err 0.00189374, control 5.28726, control_lim 1
steer:  1.00628927808409
throttle:  1.0
brake:  0.0
velocity sent:  0.036941951259807676
i 507

path_planner start
Goal before state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0, 0, 0
acceleration 0, 0, 0
Goal after state_transition
location 241.927, 7.95534, 0
rotation -1.54648, 0, 0
velocity 0.072943, -2.99911, 0
acceleration 0, 0, 0
Error: No spirals generated 
timer 1678029697, prev_timer 1678029697, new_delta_time 0
y_position 10.9992, y_desired 15.8693
err -4.87005, sum_err -1969.6, diff_err 0, -kp*err 0.0243503, -ki*sum_err 0.984799, -kd*diff_err -0, control 1.00915, control_lim 1.00915
velocity 0.036942, velocity_goal 3
err -2.96306, sum_err -1302.08, diff_err 0, -kp*err 0.0888917, -ki*sum_err 5.20833, -kd*diff_err -0, control 5.29722, control_lim 1
steer:  1.00914916711455
throttle:  1.0
brake:  0.0
velocity sent:  0.036941951259807676
i 508


*/