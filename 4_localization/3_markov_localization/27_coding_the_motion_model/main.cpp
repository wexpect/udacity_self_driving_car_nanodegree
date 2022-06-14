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
        // i is pre_pseudo_position
        float belief_state_prob = priors[i];

        float delta_position = pseudo_position - i;

        float transition_model_prob = Helpers::normpdf(delta_position, movement, control_stdev);

        float motion_model_prob = transition_model_prob * belief_state_prob;

        std::cout << "pre_pseudo_position: " << i << ", delta_position: " << delta_position << ", movement: " << movement << ", control_stdev: " << control_stdev << ", transition_model_prob: " << transition_model_prob << ", belief_state_prob: " << belief_state_prob << ", motion_model_prob: " << motion_model_prob << std::endl;

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

pseudo position: 0
pre_pseudo_position: 0, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 5, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 6, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 7, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 10, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 11, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 12, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 20, delta_position: -20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 21, delta_position: -21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -22, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -23, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -24, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 1.65867e-07
0	1.65867e-07

pseudo position: 1
pre_pseudo_position: 0, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 5, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 6, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 7, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 10, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 11, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 12, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 20, delta_position: -19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 21, delta_position: -20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -22, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -23, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 1.50359e-05
1	1.50359e-05

pseudo position: 2
pre_pseudo_position: 0, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 5, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 6, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 7, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 10, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 11, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 12, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 20, delta_position: -18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 21, delta_position: -19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -22, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.000507463
2	0.000507463

pseudo position: 3
pre_pseudo_position: 0, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 5, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 6, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 7, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 10, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 11, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 12, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 20, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 21, delta_position: -18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.00650629
3	0.00650629

pseudo position: 4
pre_pseudo_position: 0, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 5, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 6, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 7, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 10, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 11, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 12, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 20, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 21, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0333771
4	0.0333771

pseudo position: 5
pre_pseudo_position: 0, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 5, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 6, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 7, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 10, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 11, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 12, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 20, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 21, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0772117
5	0.0772117

pseudo position: 6
pre_pseudo_position: 0, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 5, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 6, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 7, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 10, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 11, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 12, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0.111111, motion_model_prob: 1.26117e-44
pre_pseudo_position: 20, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 21, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0981132
6	0.0981132

pseudo position: 7
pre_pseudo_position: 0, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 5, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 6, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 7, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 10, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 11, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 12, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 20, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0.111111, motion_model_prob: 1.26117e-44
pre_pseudo_position: 21, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 22, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.077719
7	0.077719

pseudo position: 8
pre_pseudo_position: 0, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 5, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 6, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 7, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 10, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 11, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 12, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 20, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 21, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0.111111, motion_model_prob: 1.26117e-44
pre_pseudo_position: 22, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0398834
8	0.0398834

pseudo position: 9
pre_pseudo_position: 0, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 5, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 6, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 7, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 10, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 11, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 12, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 20, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 21, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 22, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -15, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0398834
9	0.0398834

pseudo position: 10
pre_pseudo_position: 0, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 5, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 6, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 7, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 10, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 11, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 12, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 20, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 21, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 22, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -14, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.077719
10	0.077719

pseudo position: 11
pre_pseudo_position: 0, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 5, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 6, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 7, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 10, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 11, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 12, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 20, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 21, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 22, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -13, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0981132
11	0.0981132

pseudo position: 12
pre_pseudo_position: 0, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 5, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 6, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 7, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 10, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 11, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 12, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 20, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 21, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 22, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -12, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0772117
12	0.0772117

pseudo position: 13
pre_pseudo_position: 0, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 5, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 6, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 7, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 10, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 11, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 12, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 20, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 21, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 22, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -11, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0333771
13	0.0333771

pseudo position: 14
pre_pseudo_position: 0, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 5, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 6, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 7, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 10, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 11, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 12, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 20, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 21, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 22, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -10, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.00650629
14	0.00650629

pseudo position: 15
pre_pseudo_position: 0, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 5, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 6, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 7, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 10, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 11, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 12, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 20, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 21, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 22, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -9, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.000507629
15	0.000507629

pseudo position: 16
pre_pseudo_position: 0, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 5, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 6, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 7, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 10, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 11, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 12, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 20, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 21, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 22, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -8, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
position_prob: 3.00718e-05
16	3.00718e-05

pseudo position: 17
pre_pseudo_position: 0, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 5, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 6, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 7, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 10, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 11, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 12, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 20, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 21, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0.111111, motion_model_prob: 1.65191e-07
pre_pseudo_position: 22, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -7, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.000507629
17	0.000507629

pseudo position: 18
pre_pseudo_position: 0, delta_position: 18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 5, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 6, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 7, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 10, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 11, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0.111111, motion_model_prob: 6.75098e-10
pre_pseudo_position: 12, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 20, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 21, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 22, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -6, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.00650629
18	0.00650629

pseudo position: 19
pre_pseudo_position: 0, delta_position: 19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0.111111, motion_model_prob: 1.26117e-44
pre_pseudo_position: 5, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 6, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 7, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 10, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 11, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0.111111, motion_model_prob: 1.01497e-12
pre_pseudo_position: 12, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 20, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 21, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 22, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -5, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0333771
19	0.0333771

pseudo position: 20
pre_pseudo_position: 0, delta_position: 20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 5, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0.111111, motion_model_prob: 1.26117e-44
pre_pseudo_position: 6, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 7, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 10, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 11, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0.111111, motion_model_prob: 5.61363e-16
pre_pseudo_position: 12, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 20, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 21, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 22, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -4, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0772116
20	0.0772116

pseudo position: 21
pre_pseudo_position: 0, delta_position: 21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 5, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 6, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0.111111, motion_model_prob: 1.26117e-44
pre_pseudo_position: 7, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 10, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 11, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0.111111, motion_model_prob: 1.1422e-19
pre_pseudo_position: 12, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 20, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 21, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 22, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -3, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0980982
21	0.0980982

pseudo position: 22
pre_pseudo_position: 0, delta_position: 22, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 5, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 6, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 7, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 10, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 11, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0.111111, motion_model_prob: 8.54955e-24
pre_pseudo_position: 12, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 20, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 21, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0.111111, motion_model_prob: 0.0443269
pre_pseudo_position: 22, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -2, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0772116
22	0.0772116

pseudo position: 23
pre_pseudo_position: 0, delta_position: 23, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 22, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 5, delta_position: 18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 6, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 7, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 10, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 11, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0.111111, motion_model_prob: 2.35424e-28
pre_pseudo_position: 12, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 20, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 21, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0.111111, motion_model_prob: 0.0268856
pre_pseudo_position: 22, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: -1, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.0333771
23	0.0333771

pseudo position: 24
pre_pseudo_position: 0, delta_position: 24, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 1, delta_position: 23, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 2, delta_position: 22, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 3, delta_position: 21, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 4, delta_position: 20, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 5, delta_position: 19, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 6, delta_position: 18, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0.111111, motion_model_prob: 0
pre_pseudo_position: 7, delta_position: 17, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 8, delta_position: 16, movement: 1, control_stdev: 1, transition_model_prob: 0, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 9, delta_position: 15, movement: 1, control_stdev: 1, transition_model_prob: 1.09301e-43, belief_state_prob: 0.111111, motion_model_prob: 1.26117e-44
pre_pseudo_position: 10, delta_position: 14, movement: 1, control_stdev: 1, transition_model_prob: 7.99883e-38, belief_state_prob: 0.111111, motion_model_prob: 8.88759e-39
pre_pseudo_position: 11, delta_position: 13, movement: 1, control_stdev: 1, transition_model_prob: 2.14638e-32, belief_state_prob: 0.111111, motion_model_prob: 2.38487e-33
pre_pseudo_position: 12, delta_position: 12, movement: 1, control_stdev: 1, transition_model_prob: 2.11882e-27, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 13, delta_position: 11, movement: 1, control_stdev: 1, transition_model_prob: 7.6946e-23, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 14, delta_position: 10, movement: 1, control_stdev: 1, transition_model_prob: 1.02798e-18, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 15, delta_position: 9, movement: 1, control_stdev: 1, transition_model_prob: 5.05227e-15, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 16, delta_position: 8, movement: 1, control_stdev: 1, transition_model_prob: 9.13472e-12, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 17, delta_position: 7, movement: 1, control_stdev: 1, transition_model_prob: 6.07588e-09, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 18, delta_position: 6, movement: 1, control_stdev: 1, transition_model_prob: 1.48672e-06, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 19, delta_position: 5, movement: 1, control_stdev: 1, transition_model_prob: 0.00013383, belief_state_prob: 0.111111, motion_model_prob: 1.487e-05
pre_pseudo_position: 20, delta_position: 4, movement: 1, control_stdev: 1, transition_model_prob: 0.00443185, belief_state_prob: 0.111111, motion_model_prob: 0.000492428
pre_pseudo_position: 21, delta_position: 3, movement: 1, control_stdev: 1, transition_model_prob: 0.053991, belief_state_prob: 0.111111, motion_model_prob: 0.005999
pre_pseudo_position: 22, delta_position: 2, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 23, delta_position: 1, movement: 1, control_stdev: 1, transition_model_prob: 0.398942, belief_state_prob: 0, motion_model_prob: 0
pre_pseudo_position: 24, delta_position: 0, movement: 1, control_stdev: 1, transition_model_prob: 0.241971, belief_state_prob: 0, motion_model_prob: 0
position_prob: 0.00650629
24	0.00650629

sum_motion_prob: 0.999478
*/