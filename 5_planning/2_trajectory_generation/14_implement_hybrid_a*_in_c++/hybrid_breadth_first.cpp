#include <math.h>


// NOTE: added code
#include <algorithm>    // std::sort


#include <iostream>
#include <vector>
#include "hybrid_breadth_first.h"

// Initializes HBF
HBF::HBF() {}

HBF::~HBF() {}

int HBF::theta_to_stack_number(double theta){
  // Takes an angle (in radians) and returns which "stack" in the 3D 
  //   configuration space this angle corresponds to. Angles near 0 go in the 
  //   lower stacks while angles near 2 * pi go in the higher stacks.
  double new_theta = fmod( (theta + 2 * M_PI), (2 * M_PI) );
  int stack_number = (int)( round( new_theta * NUM_THETA_CELLS / (2*M_PI) ) ) % NUM_THETA_CELLS;

  return stack_number;
}

int HBF::idx(double float_num) {
  // Returns the index into the grid for continuous position. So if x is 3.621, 
  //   then this would return 3 to indicate that 3.621 corresponds to array 
  //   index 3.
  return int(floor(float_num));
}


// NOTE: added code
bool HBF::compare_state(const HBF::maze_s &s1, const HBF::maze_s &s2){
  return s1.f < s2.f;
}
double HBF::heuristic(double x, double y, vector<int> &goal){
  return fabs(goal[0] - x) + fabs(goal[1] - y);
}


// NOTE: return a list of possible next states for a range of steering angles
vector<HBF::maze_s> HBF::expand(HBF::maze_s &state, vector<int> &goal) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;
    
  int g2 = g + 1;

  vector<HBF::maze_s> next_states;

  for(double delta_i = -35; delta_i < 40; delta_i+=5) {
    // NOTE: delta_i is in degrees, delta is in radian
    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if(theta2 < 0) {
      theta2 += 2*M_PI;
    }

    double x2 = x + SPEED * cos(theta);
    double y2 = y + SPEED * sin(theta);

    HBF::maze_s state2;
    state2.g = g2;


    // NOTE: added code
    state2.f = g2 + heuristic(x2, y2, goal);


    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;

    next_states.push_back(state2);
  }

  return next_states;
}

// NOTE: 
// came_from[stack][x_idx][y_idx] = state
// start is vector<double> of {x, y, theta}
vector< HBF::maze_s> HBF::reconstruct_path(
  vector<vector<vector<HBF::maze_s>>> &came_from, vector<double> &start, 
  HBF::maze_s &final_state) {

  vector<maze_s> path = {final_state};
  
  int stack = theta_to_stack_number(final_state.theta);

  maze_s come_from_state = came_from[stack][idx(final_state.x)][idx(final_state.y)];
  
  stack = theta_to_stack_number(come_from_state.theta);
  
  double x = come_from_state.x;
  double y = come_from_state.y;

  while(x != start[0] || y != start[1]) {
    path.push_back(come_from_state);
    come_from_state = came_from[stack][idx(x)][idx(y)];
    x = come_from_state.x;
    y = come_from_state.y;
    stack = theta_to_stack_number(come_from_state.theta);
  }
  
  return path;
}

HBF::maze_path HBF::search(vector< vector<int> > &grid, vector<double> &start, 
                           vector<int> &goal) {
  // Working Implementation of breadth first search. Does NOT use a heuristic
  //   and as a result this is pretty inefficient. Try modifying this algorithm 
  //   into hybrid A* by adding heuristics appropriately.

  /**
   * TODO: Add heuristics and convert this function into hybrid A*
   */

  // NOTE: closed(stack, x_idx, y_idx), means searched
  vector<vector<vector<int>>> closed(NUM_THETA_CELLS, vector<vector<int>>( grid[0].size(), vector<int>(grid.size()) ) );

  // NOTE: which state lead to this stacked_cell
  vector<vector<vector<maze_s>>> came_from( NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())) );
  
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  
  int g = 0;

  maze_s state;
  state.g = g;


  // NOTE: added code
  state.f = g + heuristic(start[0], start[1], goal);


  state.x = start[0];
  state.y = start[1];
  state.theta = theta;

  closed[stack][idx(state.x)][idx(state.y)] = 1;

  // Udacity: Store our starting state. For other states, we will store the previous state in the path, but the starting state has no previous.
  came_from[stack][idx(state.x)][idx(state.y)] = state;

  int total_closed = 1;

  // NOTE: states in queue, to be checked
  // keeps track of the states we are searching through  
  vector<maze_s> opened = {state};
  bool finished = false;

  while(!opened.empty()) {

    // NOTE: added code, for Hybrid A*
    std::sort(opened.begin(), opened.end(), compare_state);


    maze_s current = opened[0]; // grab first elment
    opened.erase(opened.begin()); // pop first element

    int x = current.x;
    int y = current.y;

    // NOTE: check if in the same cell, does not care about theta
    if(idx(x) == goal[0] && idx(y) == goal[1]) {
      std::cout << "found path to goal in " << total_closed << " expansions" << std::endl;
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;

      return path;
    }

    // expand the current state to get a list of possible next states
    vector<maze_s> next_states = expand(current, goal);

    for(int i = 0; i < next_states.size(); ++i) {
      int g2 = next_states[i].g;
      double x2 = next_states[i].x;
      double y2 = next_states[i].y;
      double theta2 = next_states[i].theta;

      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size())) {
        // invalid cell
        continue;
      }

      int stack2 = theta_to_stack_number(theta2);

      // NOTE: a stacked_cell can only be represneted by the 1st state, not the states after
      // NOTE: the 1st state is added to the opened (queue)
      if(grid[idx(x2)][idx(y2)] == 0 && closed[stack2][idx(x2)][idx(y2)] == 0) {
        opened.push_back(next_states[i]);
        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        ++total_closed;
      }

    }

  }

  std::cout << "did not find valid path." << std::endl;
  HBF::maze_path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}