#include <algorithm>  // std::sort
#include <iostream>
#include <vector>

#include "helpers.h"

using std::vector;

// set standard deviation of control:
float control_stdev = 1.0f;

// meters vehicle moves per time step
float movement_per_timestep = 1.0f;

// number of x positions on map
int map_size = 25;

// define landmarks
vector<float> landmark_positions {5, 10, 12, 20};

// declare pseudo_range_estimator function
vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position);


int main() {    
  // step through each pseudo position x (i)
  for (int i = 0; i < map_size; ++i) {
    float pseudo_position = float(i);
    // get pseudo ranges
    vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, 
                                                         pseudo_position);
    // print to stdout
    if (pseudo_ranges.size() > 0) {
      for (int s = 0; s < pseudo_ranges.size(); ++s) {
        std::cout << "x: " << i << "\t" << pseudo_ranges[s] << std::endl;
      }
      std::cout << "-----------------------" << std::endl;
    }   
  } 

  return 0;
}

// TODO: Complete pseudo range estimator function
vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position) {
  // define pseudo observation vector
  vector<float> pseudo_ranges;
            
  // loop over number of landmarks and estimate pseudo ranges
  // YOUR CODE HERE

  // Solution 1:
  // for(int i = 0; i < landmark_positions.size(); ++i){
  //   float position = landmark_positions[i];

  //   float pseudo_range = position - pseudo_position;

  //   if(pseudo_range > 0){
  //     pseudo_ranges.push_back(pseudo_range);
  //   }
  // }

  // Solutuon 2:
  for(auto position : landmark_positions){
    // std::cout << position << std::endl;

    float pseudo_range = position - pseudo_position;

    if(pseudo_range > 0){
      pseudo_ranges.push_back(pseudo_range);
    }
  }
  

  // sort pseudo range vector
  // YOUR CODE HERE
  std::sort(pseudo_ranges.begin(), pseudo_ranges.end());
    
  return pseudo_ranges;
}

/* 
output:

x: 0	5
x: 0	10
x: 0	12
x: 0	20
-----------------------
x: 1	4
x: 1	9
x: 1	11
x: 1	19
-----------------------
x: 2	3
x: 2	8
x: 2	10
x: 2	18
-----------------------
x: 3	2
x: 3	7
x: 3	9
x: 3	17
-----------------------
x: 4	1
x: 4	6
x: 4	8
x: 4	16
-----------------------
x: 5	5
x: 5	7
x: 5	15
-----------------------
x: 6	4
x: 6	6
x: 6	14
-----------------------
x: 7	3
x: 7	5
x: 7	13
-----------------------
x: 8	2
x: 8	4
x: 8	12
-----------------------
x: 9	1
x: 9	3
x: 9	11
-----------------------
x: 10	2
x: 10	10
-----------------------
x: 11	1
x: 11	9
-----------------------
x: 12	8
-----------------------
x: 13	7
-----------------------
x: 14	6
-----------------------
x: 15	5
-----------------------
x: 16	4
-----------------------
x: 17	3
-----------------------
x: 18	2
-----------------------
x: 19	1
-----------------------

 */