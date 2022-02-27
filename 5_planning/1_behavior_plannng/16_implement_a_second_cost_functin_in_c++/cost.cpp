#include "cost.h"

#include <stdlib.h>
#include <cmath>
#include <iostream>

double inefficiency_cost(int target_speed, int intended_lane, int final_lane, 
                         const std::vector<int> &lane_speeds) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than target_speed.

  /**
   * TODO: Replace cost = 0 with an appropriate cost function.
   */
//   double cost = 0;

    double intended_speed = lane_speeds[intended_lane];
    std::cout << "intended_lane = " << intended_lane << ", intended_speed = " << intended_speed << std::endl;

    double final_speed = lane_speeds[final_lane];
    std::cout << "final_lane = " << final_lane<< ", final_speed = " << final_speed << std::endl;

    // both options work

    // Option1
    // double abs_speed_diff = abs(intended_speed - target_speed) + abs(final_speed - target_speed);
    // double cost = 1 - exp(-1 * abs_speed_diff);

    // Option2
    double cost = ( abs(intended_speed - target_speed) / double(target_speed) + abs(final_speed - target_speed) / double(target_speed) ) / 2;

    return cost;
}