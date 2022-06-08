#include <algorithm>
#include <iostream>
#include <vector>

# include <stdlib.h>

#include "helpers.h"

using std::vector;

// function to get pseudo ranges
vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position);

// observation model: calculate likelihood prob term based on landmark proximity
float observation_model(vector<float> landmark_positions, 
                        vector<float> observations, vector<float> pseudo_ranges,
                        float distance_max, float observation_stdev);

int main() {  
  // set observation standard deviation:
  float observation_stdev = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // set distance max
  float distance_max = map_size;

  // define landmarks
  vector<float> landmark_positions {5, 10, 12, 20};

  // define observations
  // vector<float> observations {5.5, 13, 15};
  vector<float> observations {5, 7, 15};

  float total_prob = 0;

  // step through each pseudo position x (i)
  for (int i = 0; i < map_size; ++i) {
    float pseudo_position = float(i);

    std::cout << "\npseudo_position " << pseudo_position << std::endl;     

    // get pseudo ranges
    vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, 
                                                         pseudo_position);

    //get observation probability
    float observation_prob = observation_model(landmark_positions, observations, 
                                               pseudo_ranges, distance_max, 
                                               observation_stdev);
    //print to stdout
    std::cout << "observation_prob " << observation_prob << std::endl; 

    total_prob += observation_prob;
  }      

  std::cout << "\ntotal_prob " << total_prob << std::endl; 

  return 0;
}

// TODO: Complete the observation model function
// calculates likelihood prob term based on landmark proximity
float observation_model(vector<float> landmark_positions, 
                        vector<float> observations, vector<float> pseudo_ranges, 
                        float distance_max, float observation_stdev) {
  // float distance_prob;  
  
  // YOUR CODE HERE
  if( observations.size() == 0){
    return 0;
  }

  float product_of_prob = 1;

  int pseudo_ranges_size = pseudo_ranges.size();
  for(int i = 0; i < observations.size(); ++i){
    float observation = observations[i];

    // set float to infinity
    // https://stackoverflow.com/questions/8690567/setting-an-int-to-infinity-in-c
    float pseudo_range = std::numeric_limits<const float>::infinity();
    if(i < pseudo_ranges_size) {
      pseudo_range = pseudo_ranges[i];
    }

    float prob = Helpers::normpdf(observation, pseudo_range, observation_stdev);
    product_of_prob *= prob;
    std::cout << "observation " << observation << ", pseudo_range " << pseudo_range << ", prob " << prob << ", product_of_prob " << product_of_prob << std::endl; 
  }

  return product_of_prob;
 


  /* Udacity solution:

  // initialize observation probability
  float distance_prob = 1.0f;

  // run over current observation vector
  for (int z=0; z< observations.size(); ++z) {
    // define min distance
    float pseudo_range_min;
        
    // check, if distance vector exists
    if (pseudo_ranges.size() > 0) {
      // set min distance
      pseudo_range_min = pseudo_ranges[0];
      // remove this entry from pseudo_ranges-vector
      pseudo_ranges.erase(pseudo_ranges.begin());
    } else {  // no or negative distances: set min distance to a large number
        pseudo_range_min = std::numeric_limits<const float>::infinity();
    }

    std::cout << "observations[z] " << observations[z] << ", pseudo_range_min " << pseudo_range_min << std::endl; 

    // estimate the probability for observation model, this is our likelihood 
    distance_prob *= Helpers::normpdf(observations[z], pseudo_range_min,
                                      observation_stdev);
  }

  return distance_prob;
  */
  
}

vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position) {
  // define pseudo observation vector
  vector<float> pseudo_ranges;
            
  // loop over number of landmarks and estimate pseudo ranges
  for (int l=0; l< landmark_positions.size(); ++l) {
    // estimate pseudo range for each single landmark 
    // and the current state position pose_i:
    float range_l = landmark_positions[l] - pseudo_position;

    // check if distances are positive: 
    if (range_l > 0.0f) {
      pseudo_ranges.push_back(range_l);
    }
  }

  // sort pseudo range vector
  sort(pseudo_ranges.begin(), pseudo_ranges.end());

  return pseudo_ranges;
}


/* 
vector<float> observations {5.5, 13, 15};

pseudo_position 0
observation 5.5, pseudo_range 5, prob 0.352065, product_of_prob 0.352065
observation 13, pseudo_range 10, prob 0.00443185, product_of_prob 0.0015603
observation 15, pseudo_range 12, prob 0.00443185, product_of_prob 6.91501e-06
observation_prob 6.91501e-06

pseudo_position 1
observation 5.5, pseudo_range 4, prob 0.129518, product_of_prob 0.129518
observation 13, pseudo_range 9, prob 0.00013383, product_of_prob 1.73334e-05
observation 15, pseudo_range 11, prob 0.00013383, product_of_prob 2.31973e-09
observation_prob 2.31973e-09

pseudo_position 2
observation 5.5, pseudo_range 3, prob 0.0175283, product_of_prob 0.0175283
observation 13, pseudo_range 8, prob 1.48672e-06, product_of_prob 2.60597e-08
observation 15, pseudo_range 10, prob 1.48672e-06, product_of_prob 3.87434e-14
observation_prob 3.87434e-14

pseudo_position 3
observation 5.5, pseudo_range 2, prob 0.000872683, product_of_prob 0.000872683
observation 13, pseudo_range 7, prob 6.07588e-09, product_of_prob 5.30232e-12
observation 15, pseudo_range 9, prob 6.07588e-09, product_of_prob 3.22163e-20
observation_prob 3.22163e-20

pseudo_position 4
observation 5.5, pseudo_range 1, prob 1.59837e-05, product_of_prob 1.59837e-05
observation 13, pseudo_range 6, prob 9.13472e-12, product_of_prob 1.46007e-16
observation 15, pseudo_range 8, prob 9.13472e-12, product_of_prob 1.33373e-27
observation_prob 1.33373e-27

pseudo_position 5
observation 5.5, pseudo_range 5, prob 0.352065, product_of_prob 0.352065
observation 13, pseudo_range 7, prob 6.07588e-09, product_of_prob 2.13911e-09
observation 15, pseudo_range 15, prob 0.398942, product_of_prob 8.53381e-10
observation_prob 8.53381e-10

pseudo_position 6
observation 5.5, pseudo_range 4, prob 0.129518, product_of_prob 0.129518
observation 13, pseudo_range 6, prob 9.13472e-12, product_of_prob 1.18311e-12
observation 15, pseudo_range 14, prob 0.241971, product_of_prob 2.86277e-13
observation_prob 2.86277e-13

pseudo_position 7
observation 5.5, pseudo_range 3, prob 0.0175283, product_of_prob 0.0175283
observation 13, pseudo_range 5, prob 5.05227e-15, product_of_prob 8.85577e-17
observation 15, pseudo_range 13, prob 0.053991, product_of_prob 4.78132e-18
observation_prob 4.78132e-18

pseudo_position 8
observation 5.5, pseudo_range 2, prob 0.000872683, product_of_prob 0.000872683
observation 13, pseudo_range 4, prob 1.02798e-18, product_of_prob 8.97098e-22
observation 15, pseudo_range 12, prob 0.00443185, product_of_prob 3.9758e-24
observation_prob 3.9758e-24

pseudo_position 9
observation 5.5, pseudo_range 1, prob 1.59837e-05, product_of_prob 1.59837e-05
observation 13, pseudo_range 3, prob 7.6946e-23, product_of_prob 1.22988e-27
observation 15, pseudo_range 11, prob 0.00013383, product_of_prob 1.64596e-31
observation_prob 1.64596e-31

pseudo_position 10
observation 5.5, pseudo_range 2, prob 0.000872683, product_of_prob 0.000872683
observation 13, pseudo_range 10, prob 0.00443185, product_of_prob 3.8676e-06
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 11
observation 5.5, pseudo_range 1, prob 1.59837e-05, product_of_prob 1.59837e-05
observation 13, pseudo_range 9, prob 0.00013383, product_of_prob 2.13911e-09
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 12
observation 5.5, pseudo_range 8, prob 0.0175283, product_of_prob 0.0175283
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 13
observation 5.5, pseudo_range 7, prob 0.129518, product_of_prob 0.129518
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 14
observation 5.5, pseudo_range 6, prob 0.352065, product_of_prob 0.352065
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 15
observation 5.5, pseudo_range 5, prob 0.352065, product_of_prob 0.352065
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 16
observation 5.5, pseudo_range 4, prob 0.129518, product_of_prob 0.129518
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 17
observation 5.5, pseudo_range 3, prob 0.0175283, product_of_prob 0.0175283
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 18
observation 5.5, pseudo_range 2, prob 0.000872683, product_of_prob 0.000872683
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 19
observation 5.5, pseudo_range 1, prob 1.59837e-05, product_of_prob 1.59837e-05
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 20
observation 5.5, pseudo_range inf, prob 0, product_of_prob 0
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 21
observation 5.5, pseudo_range inf, prob 0, product_of_prob 0
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 22
observation 5.5, pseudo_range inf, prob 0, product_of_prob 0
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 23
observation 5.5, pseudo_range inf, prob 0, product_of_prob 0
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 24
observation 5.5, pseudo_range inf, prob 0, product_of_prob 0
observation 13, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

total_prob 6.91819e-06
*/



/* 
vector<float> observations {5, 7, 15};

pseudo_position 0
observation 5, pseudo_range 5, prob 0.398942, product_of_prob 0.398942
observation 7, pseudo_range 10, prob 0.00443185, product_of_prob 0.00176805
observation 15, pseudo_range 12, prob 0.00443185, product_of_prob 7.83574e-06
observation_prob 7.83574e-06

pseudo_position 1
observation 5, pseudo_range 4, prob 0.241971, product_of_prob 0.241971
observation 7, pseudo_range 9, prob 0.053991, product_of_prob 0.0130642
observation 15, pseudo_range 11, prob 0.00013383, product_of_prob 1.74839e-06
observation_prob 1.74839e-06

pseudo_position 2
observation 5, pseudo_range 3, prob 0.053991, product_of_prob 0.053991
observation 7, pseudo_range 8, prob 0.241971, product_of_prob 0.0130642
observation 15, pseudo_range 10, prob 1.48672e-06, product_of_prob 1.94229e-08
observation_prob 1.94229e-08

pseudo_position 3
observation 5, pseudo_range 2, prob 0.00443185, product_of_prob 0.00443185
observation 7, pseudo_range 7, prob 0.398942, product_of_prob 0.00176805
observation 15, pseudo_range 9, prob 6.07588e-09, product_of_prob 1.07425e-11
observation_prob 1.07425e-11

pseudo_position 4
observation 5, pseudo_range 1, prob 0.00013383, product_of_prob 0.00013383
observation 7, pseudo_range 6, prob 0.241971, product_of_prob 3.2383e-05
observation 15, pseudo_range 8, prob 9.13472e-12, product_of_prob 2.9581e-16
observation_prob 2.9581e-16

pseudo_position 5
observation 5, pseudo_range 5, prob 0.398942, product_of_prob 0.398942
observation 7, pseudo_range 7, prob 0.398942, product_of_prob 0.159155
observation 15, pseudo_range 15, prob 0.398942, product_of_prob 0.0634936
observation_prob 0.0634936

pseudo_position 6
observation 5, pseudo_range 4, prob 0.241971, product_of_prob 0.241971
observation 7, pseudo_range 6, prob 0.241971, product_of_prob 0.0585498
observation 15, pseudo_range 14, prob 0.241971, product_of_prob 0.0141673
observation_prob 0.0141673

pseudo_position 7
observation 5, pseudo_range 3, prob 0.053991, product_of_prob 0.053991
observation 7, pseudo_range 5, prob 0.053991, product_of_prob 0.00291502
observation 15, pseudo_range 13, prob 0.053991, product_of_prob 0.000157385
observation_prob 0.000157385

pseudo_position 8
observation 5, pseudo_range 2, prob 0.00443185, product_of_prob 0.00443185
observation 7, pseudo_range 4, prob 0.00443185, product_of_prob 1.96413e-05
observation 15, pseudo_range 12, prob 0.00443185, product_of_prob 8.70472e-08
observation_prob 8.70472e-08

pseudo_position 9
observation 5, pseudo_range 1, prob 0.00013383, product_of_prob 0.00013383
observation 7, pseudo_range 3, prob 0.00013383, product_of_prob 1.79105e-08
observation 15, pseudo_range 11, prob 0.00013383, product_of_prob 2.39697e-12
observation_prob 2.39697e-12

pseudo_position 10
observation 5, pseudo_range 2, prob 0.00443185, product_of_prob 0.00443185
observation 7, pseudo_range 10, prob 0.00443185, product_of_prob 1.96413e-05
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 11
observation 5, pseudo_range 1, prob 0.00013383, product_of_prob 0.00013383
observation 7, pseudo_range 9, prob 0.053991, product_of_prob 7.22562e-06
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 12
observation 5, pseudo_range 8, prob 0.00443185, product_of_prob 0.00443185
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 13
observation 5, pseudo_range 7, prob 0.053991, product_of_prob 0.053991
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 14
observation 5, pseudo_range 6, prob 0.241971, product_of_prob 0.241971
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 15
observation 5, pseudo_range 5, prob 0.398942, product_of_prob 0.398942
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 16
observation 5, pseudo_range 4, prob 0.241971, product_of_prob 0.241971
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 17
observation 5, pseudo_range 3, prob 0.053991, product_of_prob 0.053991
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 18
observation 5, pseudo_range 2, prob 0.00443185, product_of_prob 0.00443185
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 19
observation 5, pseudo_range 1, prob 0.00013383, product_of_prob 0.00013383
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 20
observation 5, pseudo_range inf, prob 0, product_of_prob 0
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 21
observation 5, pseudo_range inf, prob 0, product_of_prob 0
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 22
observation 5, pseudo_range inf, prob 0, product_of_prob 0
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 23
observation 5, pseudo_range inf, prob 0, product_of_prob 0
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

pseudo_position 24
observation 5, pseudo_range inf, prob 0, product_of_prob 0
observation 7, pseudo_range inf, prob 0, product_of_prob 0
observation 15, pseudo_range inf, prob 0, product_of_prob 0
observation_prob 0

total_prob 0.0778281
*/

