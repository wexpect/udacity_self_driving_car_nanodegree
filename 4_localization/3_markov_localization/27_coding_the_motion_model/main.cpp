#include <iostream>
#include <vector>

#include "helpers.h"

using std::vector;

vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev);

float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev);

int main() {
  // set standard deviation of control:
  float control_stdev = 1.0f;

  // set standard deviation of position:
  float position_stdev = 1.0f;

  // meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // initialize landmarks
  vector<float> landmark_positions {5, 10, 20};
    
  // initialize priors
  vector<float> priors = initialize_priors(map_size, landmark_positions,
                                           position_stdev);

  for(int i = 0; i < priors.size(); ++i){
    std::cout << i << ", " << priors[i] << std::endl;
  }


  float sum_motion_prob = 0;

  // step through each pseudo position x (i)    
  for (float i = 0; i < map_size; ++i) {
    float pseudo_position = i;

    // get the motion model probability for each x position
    float motion_prob = motion_model(pseudo_position, movement_per_timestep,
                                     priors, map_size, control_stdev);     

    // print to stdout
    std::cout << pseudo_position << "\t" << motion_prob << std::endl;

    sum_motion_prob += motion_prob;
  }    

  std::cout << "sum_motion_prob: " << sum_motion_prob << std::endl;

  return 0;
}

// TODO: implement the motion model: calculates prob of being at 
// an estimated position at time t
float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev) {
    // initialize probability
    float position_prob = 0.0f;
    
    // YOUR CODE HERE
    std::cout << "\npseudo position: " << pseudo_position << std::endl;

    for(int i = 0; i < map_size; ++i){
        float belief_state_prob = priors[i];

        float x = pseudo_position - i;

        float transition_model_prob = Helpers::normpdf(x, movement, control_stdev);

        float motion_model_prob = transition_model_prob * belief_state_prob;

        std::cout << "prior position: " << i << ", x: " << x << ", movement: " << movement << ", control_stdev: " << control_stdev << ", transition_model_prob: " << transition_model_prob << ", belief_state_prob: " << belief_state_prob << ", motion_model_prob: " << motion_model_prob << std::endl;

        position_prob += motion_model_prob;
    }

    std::cout << "position_prob: " << position_prob << std::endl;    

    return position_prob;
}

// initialize priors assuming vehicle at landmark +/- 1.0 meters position stdev
vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                     float position_stdev) {

  // set all priors to 0.0
  vector<float> priors(map_size, 0.0);

  // set each landmark positon +/-1 to 1.0/9.0 (9 possible postions)
  float norm_term = landmark_positions.size() * (position_stdev * 2 + 1);
  for (int i=0; i < landmark_positions.size(); ++i) {
    for (float j=1; j <= position_stdev; ++j) {
      priors.at(int(j+landmark_positions[i]+map_size)%map_size) += 1.0/norm_term;
      priors.at(int(-j+landmark_positions[i]+map_size)%map_size) += 1.0/norm_term;
    }
    priors.at(landmark_positions[i]) += 1.0/norm_term;
  }
  
  return priors;
}


/* 
priors:
0, 0
1, 0
2, 0
3, 0
4, 0.111111
5, 0.111111
6, 0.111111
7, 0
8, 0
9, 0.111111
10, 0.111111
11, 0.111111
12, 0
13, 0
14, 0
15, 0
16, 0
17, 0
18, 0
19, 0.111111
20, 0.111111
21, 0.111111
22, 0
23, 0
24, 0


pseudo_position     motion_prob: 
0       1.65867e-07
1       1.50359e-05
2       0.000507463
3       0.00650629
4       0.0333771
5       0.0772117
6       0.0981132
7       0.077719
8       0.0398834
9       0.0398834
10      0.077719
11      0.0981132
12      0.0772117
13      0.0333771
14      0.00650629
15      0.000507629
16      3.00718e-05
17      0.000507629
18      0.00650629
19      0.0333771
20      0.0772116
21      0.0980982
22      0.0772116
23      0.0333771
24      0.00650629

sum_motion_prob: 0.999478
*/