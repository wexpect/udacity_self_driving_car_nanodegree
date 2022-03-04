// g++ -std=c++11 *.cpp -o output.out

#include "cost.h"

#include <iostream>
#include <vector>

using std::cout;
using std::endl;

int main() {
  // Target speed of our vehicle
  int target_speed = 10;
  cout << "target_speed = " << target_speed << endl;

  // Lane speeds for each lane
  std::vector<int> lane_speeds = {6, 7, 8, 9};

//   int lane_speeds[] = {6, 7, 8, 9};
    
  // Test cases used for grading - do not change.
  double cost;

  cout << "Costs for (intended_lane, final_lane):" << endl;
  cout << "---------------------------------------------------------" << endl;

  cost = inefficiency_cost(target_speed, 3, 3, lane_speeds);
  cout << "The cost is " << cost << " for " << "(3, 3)" << endl;

  cost = inefficiency_cost(target_speed, 2, 3, lane_speeds);
  cout << "The cost is " << cost << " for " << "(2, 3)" << endl;

  cost = inefficiency_cost(target_speed, 2, 2, lane_speeds);
  cout << "The cost is " << cost << " for " << "(2, 2)" << endl;

  cost = inefficiency_cost(target_speed, 1, 2, lane_speeds);
  cout << "The cost is " << cost << " for " << "(1, 2)" << endl;

  cost = inefficiency_cost(target_speed, 1, 1, lane_speeds);
  cout << "The cost is " << cost << " for " << "(1, 1)" << endl;

  cost = inefficiency_cost(target_speed, 0, 1, lane_speeds);
  cout << "The cost is " << cost << " for " << "(0, 1)" << endl;

  cost = inefficiency_cost(target_speed, 0, 0, lane_speeds);
  cout << "The cost is " << cost << " for " << "(0, 0)" << endl;
    
  return 0;
}

/* 
target_speed = 10
Costs for (intended_lane, final_lane):
---------------------------------------------------------
intended_lane = 3, intended_speed = 9
final_lane = 3, final_speed = 9
The cost is 0 for (3, 3)
intended_lane = 2, intended_speed = 8
final_lane = 3, final_speed = 9
The cost is 0 for (2, 3)
intended_lane = 2, intended_speed = 8
final_lane = 2, final_speed = 8
The cost is 0 for (2, 2)
intended_lane = 1, intended_speed = 7
final_lane = 2, final_speed = 8
The cost is 0 for (1, 2)
intended_lane = 1, intended_speed = 7
final_lane = 1, final_speed = 7
The cost is 0 for (1, 1)
intended_lane = 0, intended_speed = 6
final_lane = 1, final_speed = 7
The cost is 0 for (0, 1)
intended_lane = 0, intended_speed = 6
final_lane = 0, final_speed = 6
The cost is 0 for (0, 0)


(env) Ruis-MacBook-Pro-15:16_implement_a_second_cost_functin_in_c++ ruiwang$ g++ -std=c++11 *.cpp -o output.out
(env) Ruis-MacBook-Pro-15:16_implement_a_second_cost_functin_in_c++ ruiwang$ ./output.out
target_speed = 10
Costs for (intended_lane, final_lane):
---------------------------------------------------------
intended_lane = 3, intended_speed = 9
final_lane = 3, final_speed = 9
The cost is 0.1 for (3, 3)
intended_lane = 2, intended_speed = 8
final_lane = 3, final_speed = 9
The cost is 0.15 for (2, 3)
intended_lane = 2, intended_speed = 8
final_lane = 2, final_speed = 8
The cost is 0.2 for (2, 2)
intended_lane = 1, intended_speed = 7
final_lane = 2, final_speed = 8
The cost is 0.25 for (1, 2)
intended_lane = 1, intended_speed = 7
final_lane = 1, final_speed = 7
The cost is 0.3 for (1, 1)
intended_lane = 0, intended_speed = 6
final_lane = 1, final_speed = 7
The cost is 0.35 for (0, 1)
intended_lane = 0, intended_speed = 6
final_lane = 0, final_speed = 6
The cost is 0.4 for (0, 0)
 */