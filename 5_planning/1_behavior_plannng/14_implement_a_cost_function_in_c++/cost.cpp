#include "cost.h"
#include <cmath>
// #include <math.h>
#include <stdlib.h>

double goal_distance_cost(int goal_lane, int intended_lane, int final_lane, 
                          double distance_to_goal) {
    // The cost increases with both the distance of intended lane from the goal
    //   and the distance of the final lane from the goal. The cost of being out 
    //   of the goal lane also becomes larger as the vehicle approaches the goal.

    /**
     * TODO: Replace cost = 0 with an appropriate cost function.
     */
    // double cost = 0;

    double abs_dist = abs(intended_lane - goal_lane) + abs(final_lane - goal_lane);

    double cost = 1 - exp(-1 * abs_dist / distance_to_goal);

    return cost;
}