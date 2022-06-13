#include <algorithm>
#include <iostream>
#include <vector>

#include "helpers.h"

using std::vector;
using std::cout;
using std::endl;

using std::max_element;

vector<float> initialize_priors(int map_size, vector<float> landmark_positions,
                                float position_stdev);

float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev);

// function to get pseudo ranges
vector<float> pseudo_range_estimator(vector<float> landmark_positions, 
                                     float pseudo_position);

// observation model: calculate likelihood prob term based on landmark proximity
float observation_model(vector<float> landmark_positions, 
                        vector<float> observations, vector<float> pseudo_ranges,
                        float distance_max, float observation_stdev);


int main() {  
  // set standard deviation of control
  float control_stdev = 1.0f;

  // set standard deviation of position
  float position_stdev = 1.0f;

  // meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  // set observation standard deviation
  float observation_stdev = 1.0f;

  // number of x positions on map
  int map_size = 25;

  // set distance max
  float distance_max = map_size;

  // define landmarks
  vector<float> landmark_positions {3, 9, 14, 23};

  // define observations vector, each inner vector represents a set 
  //   of observations for a time step
  vector<vector<float> > sensor_obs {{1,7,12,21}, {0,6,11,20}, {5,10,19},
                                     {4,9,18}, {3,8,17}, {2,7,16}, {1,6,15}, 
                                     {0,5,14}, {4,13}, {3,12}, {2,11}, {1,10},
                                     {0,9}, {8}, {7}, {6}, {5}, {4}, {3}, {2},
                                     {1}, {0}, {}, {}, {}};

  /**
   * TODO: initialize priors
   */
  vector<float> priors = initialize_priors(map_size, landmark_positions, position_stdev);

  // UNCOMMENT TO SEE THIS STEP OF THE FILTER
  cout << "-----------PRIORS INIT--------------" << endl;
  for (int p = 0; p < priors.size(); ++p){
   cout << priors[p] << endl;
  }  


  // initialize posteriors
  vector<float> posteriors(map_size, 0.0);

  // specify time steps
  int time_steps = sensor_obs.size();
    
  // declare observations vector
  vector<float> observations;
    
  // cycle through time steps
  for (int t = 0; t < time_steps; ++t) {
    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    cout << "---------------TIME STEP---------------" << endl;
    cout << "t = " << t << endl;

    if (!sensor_obs[t].empty()) {
      observations = sensor_obs[t]; 
    } else {
      // NOTE: set to a large value, so observation model prob is 0
      observations = {float(distance_max)};
    }


    cout << "landmark_positions ";
    for (auto pos : landmark_positions) {
      cout << pos << " ";  
    }       
    cout << endl;

    cout << "observations ";
    for (auto obs : observations) {
      cout << obs << " ";  
    }       
    cout << endl;


    cout << "pseudo_position-----------Motion----------Observation-----------PRODUCT--" << endl;
    // step through each pseudo position x (i)
    for (unsigned int i = 0; i < map_size; ++i) {
      float pseudo_position = float(i);

      /**
       * TODO: get the motion model probability for each x position
       */
      float motion_prob = motion_model(pseudo_position, movement_per_timestep, priors, map_size, control_stdev); 

      /**
       * TODO: get pseudo ranges
       */
      vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, pseudo_position);

      /**
       * TODO: get observation probability
       */
      float observation_prob = observation_model(landmark_positions, observations, pseudo_ranges, distance_max, observation_stdev);

      /**
       * TODO: calculate the ith posterior and pass to posteriors vector
       */
      posteriors[i] = observation_prob * motion_prob;

      // UNCOMMENT TO SEE THIS STEP OF THE FILTER
      cout << pseudo_position << "\t\t\t" << motion_prob << "\t\t" << observation_prob << "\t\t" 
          << "\t" << posteriors[i] << endl;   
    } 
        
    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    // cout << "----------RAW posteriors---------------" << endl;
    // for (int p = 0; p < posteriors.size(); ++p) {
    //  cout << posteriors[p] << endl;
    // }

    /**
     * TODO: normalize posteriors (see helpers.h for a helper function)
     */
    posteriors = Helpers::normalize_vector(posteriors);

    // print to stdout
    // NOTE: I do not think the logic of this print out makes sense
    // cout << posteriors[t] << "\t" << priors[t] << endl;

    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    cout << "----------NORMALIZED posteriors---------------" << endl;

    /**
     * TODO: update priors
     */
    priors = posteriors;

    // UNCOMMENT TO SEE THIS STEP OF THE FILTER
    // print posteriors vectors to stdout
    for (int p = 0; p < posteriors.size(); ++p) {
      cout << posteriors[p] << endl;  
    } 

    int max_prob_idx = max_element(posteriors.begin(), posteriors.end()) - posteriors.begin();
    float max_prob = posteriors[max_prob_idx];
    cout << "max_prob_idx " << max_prob_idx << ", max_prob " << max_prob << endl;
  
  }

  return 0;
}

// observation model: calculate likelihood prob term based on landmark proximity
float observation_model(vector<float> landmark_positions, 
                        vector<float> observations, vector<float> pseudo_ranges, 
                        float distance_max, float observation_stdev) {
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

    // estimate the probability for observation model, this is our likelihood 
    distance_prob *= Helpers::normpdf(observations[z], pseudo_range_min,
                                      observation_stdev);
  }

  return distance_prob;
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

// motion model: calculates prob of being at an estimated position at time t
float motion_model(float pseudo_position, float movement, vector<float> priors,
                   int map_size, int control_stdev) {
  // initialize probability
  float position_prob = 0.0f;

  // loop over state space for all possible positions x (convolution):
  for (float j=0; j< map_size; ++j) {
    
    // float next_pseudo_position = j;
    // NOTE: based on the course video at Lecture 26, this should be pre_pseudo_position
    float pre_pseudo_position = j;

    // distance from i to j
    // float distance_ij = pseudo_position-next_pseudo_position;
    float distance_ij = pseudo_position - pre_pseudo_position;

    // transition probabilities:
    float transition_prob = Helpers::normpdf(distance_ij, movement, 
                                             control_stdev);
    // estimate probability for the motion model, this is our prior
    position_prob += transition_prob * priors[j];
  }

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
-----------PRIORS INIT--------------
0
0
0.0833333
0.0833333
0.0833333
0
0
0
0.0833333
0.0833333
0.0833333
0
0
0.0833333
0.0833333
0.0833333
0
0
0
0
0
0
0.0833333
0.0833333
0.0833333
---------------TIME STEP---------------
t = 0
landmark_positions 3 9 14 23 
observations 1 7 12 21 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			0.000380597		8.49737e-06			3.23407e-09
1			0.00487972		0.00342808			1.67281e-05
2			0.0250328		0.0253303			0.000634088
3			0.0579087		0			0
4			0.0735738		0			0
5			0.0579199		0			0
6			0.0254134		0			0
7			0.00975944		0			0
8			0.0254134		0			0
9			0.0579201		0			0
10			0.0735851		0			0
11			0.0582893		0			0
12			0.0299125		0			0
13			0.0299125		0			0
14			0.0582893		0			0
15			0.0735849		0			0
16			0.0579088		0			0
17			0.0250328		0			0
18			0.00487984		0			0
19			0.000391874		0			0
20			0.000391874		0			0
21			0.00487984		0			0
22			0.0250328		0			0
23			0.0579087		0			0
24			0.0735736		0			0
----------NORMALIZED posteriors---------------
4.96923e-06
0.0257031
0.974292
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 2, max_prob 0.974292
---------------TIME STEP---------------
t = 1
landmark_positions 3 9 14 23 
observations 0 6 11 20 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			0.00570685		3.8578e-10			2.20159e-12
1			0.0588243		8.49737e-06			4.99852e-07
2			0.246005		0.00342808			0.000843327
3			0.394906		0			0
4			0.237138		0			0
5			0.0527169		0			0
6			0.00432135		0			0
7			0.000130428		0			0
8			1.44866e-06		0			0
9			5.91992e-09		0			0
10			8.90001e-12		0			0
11			4.92241e-15		0			0
12			1.00155e-18		0			0
13			7.49679e-23		0			0
14			2.06435e-27		0			0
15			2.0912e-32		0			0
16			7.79319e-38		0			0
17			1.06499e-43		0			0
18			0		0			0
19			0		0			0
20			0		0			0
21			0		0			0
22			0		0			0
23			0		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
2.60905e-09
0.000592363
0.999408
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 2, max_prob 0.999408
---------------TIME STEP---------------
t = 2
landmark_positions 3 9 14 23 
observations 5 10 19 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			0.00446121		1.94229e-08			8.66493e-11
1			0.0541023		1.45384e-12			7.86559e-14
2			0.242064		5.41794e-18			1.31149e-18
3			0.398849		0.0141673			0.00565064
4			0.241859		0.0634936			0.0153565
5			0.0539616		0.0141673			0.000764493
6			0.0044293		0.000157385			6.97106e-07
7			0.000133752		8.70472e-08			1.16427e-11
8			1.48584e-06		2.39697e-12			3.56152e-18
9			6.07229e-09		0			0
10			9.12931e-12		0			0
11			5.04928e-15		0			0
12			1.02737e-18		0			0
13			7.69004e-23		0			0
14			2.11756e-27		0			0
15			2.14511e-32		0			0
16			7.99409e-38		0			0
17			1.09301e-43		0			0
18			0		0			0
19			0		0			0
20			0		0			0
21			0		0			0
22			0		0			0
23			0		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
3.97979e-09
3.61265e-12
6.02363e-17
0.259533
0.705322
0.035113
3.20179e-05
5.34748e-10
1.6358e-16
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 4, max_prob 0.705322
---------------TIME STEP---------------
t = 3
landmark_positions 3 9 14 23 
observations 4 9 18 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			3.57831e-05		1.29189e-05			4.6228e-10
1			0.00124466		1.94229e-08			2.41748e-11
2			0.017143		1.45384e-12			2.49231e-14
3			0.101036		0.000157385			1.59015e-05
4			0.276102		0.0141673			0.00391163
5			0.35268		0.0634936			0.022393
6			0.198696		0.0141673			0.00281499
7			0.0477403		0.000157385			7.51361e-06
8			0.00506415		8.70472e-08			4.4082e-10
9			0.000252124		0			0
10			5.8913e-06		0			0
11			6.07784e-08		0			0
12			2.6746e-10		0			0
13			5.19643e-13		0			0
14			4.7385e-16		0			0
15			2.02799e-19		0			0
16			3.83202e-23		0			0
17			3.0886e-27		0			0
18			1.09909e-31		0			0
19			1.83566e-36		0			0
20			1.43899e-41		0			0
21			0		0			0
22			0		0			0
23			0		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
1.58625e-08
8.29523e-10
8.55201e-13
0.000545639
0.134222
0.768382
0.0965923
0.000257819
1.51261e-08
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 5, max_prob 0.768382
---------------TIME STEP---------------
t = 4
landmark_positions 3 9 14 23 
observations 3 8 17 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			2.81126e-07		0.000427817			1.2027e-10
1			2.15306e-05		1.29189e-05			2.78153e-10
2			0.000727292		1.94229e-08			1.41261e-11
3			0.0107971		8.70472e-08			9.39856e-10
4			0.0746093		0.000157385			1.17424e-05
5			0.244821		0.0141673			0.00346847
6			0.362434		0.0634936			0.0230122
7			0.231772		0.0141673			0.0032836
8			0.065556		0.000157385			1.03175e-05
9			0.00870082		0			0
10			0.000545038		0			0
11			1.52136e-05		0			0
12			1.82847e-07		0			0
13			9.79232e-10		0			0
14			2.47519e-12		0			0
15			2.93581e-15		0			0
16			1.5401e-18		0			0
17			3.48887e-22		0			0
18			3.55921e-26		0			0
19			1.71224e-30		0			0
20			3.7591e-35		0			0
21			3.45298e-40		0			0
22			1.4013e-45		0			0
23			0		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
4.03777e-09
9.33827e-09
4.74246e-10
3.15532e-08
0.00039422
0.116445
0.772576
0.110238
0.000346384
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 6, max_prob 0.772576
---------------TIME STEP---------------
t = 5
landmark_positions 3 9 14 23 
observations 2 7 16 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			2.78819e-09		0.000705351			1.96665e-12
1			2.3461e-07		0.000427817			1.0037e-10
2			1.84867e-05		1.29189e-05			2.38829e-10
3			0.000640918		2.39697e-12			1.53626e-15
4			0.00982106		8.70472e-08			8.54896e-10
5			0.0705342		0.000157385			1.1101e-05
6			0.239444		0.0141673			0.00339229
7			0.363104		0.0634936			0.0230548
8			0.237292		0.0141673			0.0033618
9			0.0690409		0			0
10			0.00947521		0			0
11			0.000610828		0			0
12			1.74377e-05		0			0
13			2.14945e-07		0			0
14			1.19183e-09		0			0
15			3.11549e-12		0			0
16			3.72187e-15		0			0
17			1.86341e-18		0			0
18			3.64559e-22		0			0
19			2.68865e-26		0			0
20			7.36292e-31		0			0
21			7.44355e-36		0			0
22			2.77191e-41		0			0
23			0		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
6.59507e-11
3.36587e-09
8.00903e-09
5.15178e-14
2.86685e-08
0.000372268
0.113759
0.773132
0.112736
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 7, max_prob 0.773132
---------------TIME STEP---------------
t = 6
landmark_positions 3 9 14 23 
observations 1 6 15 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			2.36527e-10		5.78987e-05			1.36946e-14
1			2.52871e-09		0.000705351			1.78363e-12
2			2.27071e-07		0.000427817			9.71446e-11
3			1.80299e-05		3.28615e-18			5.92489e-23
4			0.000627907		2.39697e-12			1.50507e-15
5			0.00967354		8.70472e-08			8.42054e-10
6			0.0699166		0.000157385			1.10038e-05
7			0.238635		0.0141673			0.00338083
8			0.363261		0.0634936			0.0230647
9			0.238194		0			0
10			0.0695253		0			0
11			0.00952838		0			0
12			0.000603268		0			0
13			1.62377e-05		0			0
14			1.72306e-07		0			0
15			6.92036e-10		0			0
16			1.03372e-12		0			0
17			5.7037e-16		0			0
18			1.1595e-19		0			0
19			8.67626e-24		0			0
20			2.38885e-28		0			0
21			2.41982e-33		0			0
22			9.01768e-39		0			0
23			1.26117e-44		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
5.17626e-13
6.74172e-11
3.67185e-09
2.23948e-21
5.68885e-14
3.18278e-08
0.000415921
0.127788
0.871796
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 8, max_prob 0.871796
---------------TIME STEP---------------
t = 7
landmark_positions 3 9 14 23 
observations 0 5 14 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			2.00429e-11		2.36619e-07			4.74253e-18
1			2.18513e-10		5.78987e-05			1.26516e-14
2			2.32251e-09		0.000705351			1.63818e-12
3			2.52567e-07		2.24299e-25			5.66506e-32
4			2.02439e-05		3.28615e-18			6.65245e-23
5			0.000705473		2.39697e-12			1.691e-15
6			0.0108637		8.70472e-08			9.45656e-10
7			0.078156		0.000157385			1.23006e-05
8			0.26203		0.0141673			0.00371227
9			0.37874		0			0
10			0.21785		0			0
11			0.0476355		0			0
12			0.00388077		0			0
13			0.000116863		0			0
14			1.29689e-06		0			0
15			5.2981e-09		0			0
16			7.96426e-12		0			0
17			4.40468e-15		0			0
18			8.96196e-19		0			0
19			6.70815e-23		0			0
20			1.84718e-27		0			0
21			1.87121e-32		0			0
22			6.97335e-38		0			0
23			9.52883e-44		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
1.27331e-15
3.39679e-12
4.39831e-10
1.521e-29
1.7861e-20
4.54012e-13
2.53897e-07
0.00330255
0.996697
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 8, max_prob 0.996697
---------------TIME STEP---------------
t = 8
landmark_positions 3 9 14 23 
observations 4 13 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			2.13299e-12		3.2383e-05			6.90726e-17
1			2.46061e-11		8.02694e-08			1.97512e-18
2			1.3733e-10		2.69274e-11			3.69793e-21
3			1.11761e-08		0.00291502			3.25785e-11
4			1.92502e-06		0.00107238			2.06435e-09
5			0.000148038		5.33905e-05			7.90385e-09
6			0.00459558		3.59743e-07			1.65323e-09
7			0.0546119		3.28043e-10			1.7915e-11
8			0.242489		4.04837e-14			9.81686e-15
9			0.398424		0.0585498			0.0233276
10			0.24135		0.159155			0.038412
11			0.0538273		0.0585498			0.00315158
12			0.00441765		0.00291502			1.28776e-05
13			0.000133393		1.96413e-05			2.62001e-09
14			1.48183e-06		0			0
15			6.05585e-09		0			0
16			9.10457e-12		0			0
17			5.03559e-15		0			0
18			1.02458e-18		0			0
19			7.66919e-23		0			0
20			2.11182e-27		0			0
21			2.13929e-32		0			0
22			7.97241e-38		0			0
23			1.09301e-43		0			0
24			0		0			0
----------NORMALIZED posteriors---------------
1.06423e-15
3.04313e-17
5.69753e-20
5.01948e-10
3.18062e-08
1.21777e-07
2.54718e-08
2.76023e-10
1.51252e-13
0.359417
0.591827
0.0485574
0.000198409
4.03674e-08
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 10, max_prob 0.591827
---------------TIME STEP---------------
t = 9
landmark_positions 3 9 14 23 
observations 3 12 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			1.15462e-13		0.00176805			2.04143e-16
1			6.66282e-12		3.2383e-05			2.15762e-16
2			1.84398e-10		8.02694e-08			1.48015e-17
3			2.3851e-09		0.00107238			2.55772e-12
4			1.67734e-08		0.00291502			4.88948e-11
5			5.81602e-07		0.00107238			6.23697e-10
6			4.90435e-05		5.33905e-05			2.61846e-09
7			0.0016722		3.59743e-07			6.01561e-10
8			0.0220347		3.28043e-10			7.22831e-12
9			0.119137		0.00291502			0.000347287
10			0.289214		0.0585498			0.0169334
11			0.334833		0.159155			0.0532904
12			0.18203		0.0585498			0.0106578
13			0.0453748		0.00291502			0.000132269
14			0.00534068		0			0
15			0.00030566		0			0
16			8.26202e-06		0			0
17			1.02522e-07		0			0
18			6.00818e-10		0			0
19			1.71207e-12		0			0
20			2.30361e-15		0			0
21			1.42112e-18		0			0
22			4.11645e-22		0			0
23			5.68665e-26		0			0
24			3.52754e-30		0			0
----------NORMALIZED posteriors---------------
2.50909e-15
2.6519e-15
1.81924e-16
3.14366e-11
6.0096e-10
7.66578e-09
3.21831e-08
7.39371e-09
8.88423e-11
0.00426846
0.208127
0.654985
0.130994
0.0016257
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 11, max_prob 0.654985
---------------TIME STEP---------------
t = 10
landmark_positions 3 9 14 23 
observations 2 11 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			5.89861e-15		0.0130642			7.70609e-17
1			2.32994e-13		0.00176805			4.11945e-16
2			5.43619e-12		3.2383e-05			1.7604e-16
3			7.83857e-11		5.33905e-05			4.18505e-15
4			7.43299e-10		0.00107238			7.97097e-13
5			1.14892e-08		0.00291502			3.34912e-11
6			8.96048e-07		0.00107238			9.60902e-10
7			4.77619e-05		5.33905e-05			2.55003e-09
8			0.00124071		3.59743e-07			4.46335e-10
9			0.0151901		1.96413e-05			2.98354e-07
10			0.0880075		0.00291502			0.000256544
11			0.24963		0.0585498			0.0146158
12			0.343677		0.159155			0.0546979
13			0.222395		0.0585498			0.0130212
14			0.0686315		0			0
15			0.0103965		0			0
16			0.000756284		0			0
17			2.57108e-05		0			0
18			4.163e-07		0			0
19			3.21884e-09		0			0
20			1.10775e-11		0			0
21			1.55128e-14		0			0
22			8.34818e-18		0			0
23			1.68126e-21		0			0
24			1.25369e-25		0			0
----------NORMALIZED posteriors---------------
9.33033e-16
4.98773e-15
2.13145e-15
5.06716e-14
9.65105e-12
4.05503e-10
1.16344e-08
3.08752e-08
5.40411e-09
3.61239e-06
0.00310617
0.176965
0.662268
0.157658
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 12, max_prob 0.662268
---------------TIME STEP---------------
t = 11
landmark_positions 3 9 14 23 
observations 1 10 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			5.28205e-16		0.0130642			6.9006e-18
1			3.8842e-15		0.0130642			5.07441e-17
2			1.19992e-13		0.00176805			2.12153e-16
3			3.93558e-12		3.59743e-07			1.4158e-18
4			8.00028e-11		5.33905e-05			4.27139e-15
5			8.93553e-10		0.00107238			9.58226e-13
6			1.08529e-08		0.00291502			3.16366e-11
7			7.11334e-07		0.00107238			7.62818e-10
8			3.86463e-05		5.33905e-05			2.06335e-09
9			0.00104174		1.79105e-08			1.8658e-11
10			0.0132637		1.96413e-05			2.60516e-07
11			0.0805155		0.00291502			0.000234705
12			0.240112		0.0585498			0.0140585
13			0.345343		0.159155			0.0549631
14			0.232714		0			0
15			0.0746897		0			0
16			0.0114708		0			0
17			0.000787609		0			0
18			2.2085e-05		0			0
19			2.38418e-07		0			0
20			9.6396e-10		0			0
21			1.4435e-12		0			0
22			7.9721e-16		0			0
23			1.62119e-19		0			0
24			1.21325e-23		0			0
----------NORMALIZED posteriors---------------
9.96382e-17
7.32697e-16
3.06329e-15
2.04428e-17
6.16749e-14
1.38359e-11
4.56802e-10
1.10144e-08
2.97928e-08
2.69405e-10
3.76161e-06
0.00338892
0.202992
0.793615
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 13, max_prob 0.793615
---------------TIME STEP---------------
t = 12
landmark_positions 3 9 14 23 
observations 0 9 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			7.74273e-17		0.00176805			1.36895e-19
1			4.14222e-16		0.0130642			5.41149e-18
2			3.93007e-15		0.0130642			5.13433e-17
3			1.43748e-13		3.28043e-10			4.71556e-23
4			4.30561e-12		3.59743e-07			1.54891e-18
5			8.08924e-11		5.33905e-05			4.31889e-15
6			8.70859e-10		0.00107238			9.3389e-13
7			1.12429e-08		0.00291502			3.27733e-11
8			7.88553e-07		0.00107238			8.45627e-10
9			4.35832e-05		2.21034e-12			9.63335e-17
10			0.00118973		1.79105e-08			2.13087e-11
11			0.0152984		1.96413e-05			3.00481e-07
12			0.093319		0.00291502			0.000272027
13			0.273834		0.0585498			0.0160329
14			0.365908		0			0
15			0.203006		0			0
16			0.0437481		0			0
17			0.00354436		0			0
18			0.000106512		0			0
19			1.18112e-06		0			0
20			4.82377e-09		0			0
21			7.25048e-12		0			0
22			4.00977e-15		0			0
23			8.15834e-19		0			0
24			6.1066e-23		0			0
----------NORMALIZED posteriors---------------
8.39579e-18
3.31886e-16
3.14888e-15
2.89205e-21
9.49946e-17
2.64877e-13
5.72754e-11
2.00998e-09
5.18622e-08
5.90813e-15
1.30686e-09
1.84285e-05
0.0166834
0.983298
0
0
0
0
0
0
0
0
0
0
0
max_prob_idx 13, max_prob 0.983298
---------------TIME STEP---------------
t = 13
landmark_positions 3 9 14 23 
observations 8 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			3.3908e-17		1.48672e-06			5.04117e-23
1			2.5444e-16		6.07588e-09			1.54595e-24
2			1.03008e-15		9.13472e-12			9.4095e-27
3			1.34846e-14		0.053991			7.28045e-16
4			6.1504e-13		0.00443185			2.72577e-15
5			1.90056e-11		0.00013383			2.54352e-15
6			3.52602e-10		1.48672e-06			5.2422e-16
7			3.44728e-09		6.07588e-09			2.09453e-17
8			4.66149e-08		9.13472e-12			4.25814e-19
9			3.79755e-06		0.00443185			1.68302e-08
10			0.000206541		0.00013383			2.76415e-08
11			0.00526304		1.48672e-06			7.82467e-09
12			0.0571335		6.07588e-09			3.47136e-10
13			0.24459		9.13472e-12			2.23426e-12
14			0.396317		0.241971			0.0958971
15			0.23883		0.398942			0.0952795
16			0.0531632		0.241971			0.0128639
17			0.00436006		0.053991			0.000235404
18			0.00013162		0.00443185			5.83319e-07
19			1.46199e-06		0.00013383			1.95658e-10
20			5.97456e-09		1.48672e-06			8.88249e-15
21			8.98224e-12		6.07588e-09			5.4575e-20
22			4.96791e-15		9.13472e-12			4.53804e-26
23			1.01081e-18		0			0
24			7.56609e-23		0			0
----------NORMALIZED posteriors---------------
2.46782e-22
7.56793e-24
4.60625e-26
3.56402e-15
1.33435e-14
1.24514e-14
2.56623e-15
1.02534e-16
2.0845e-18
8.23892e-08
1.35314e-07
3.83043e-08
1.69934e-09
1.09374e-11
0.469448
0.466424
0.0629731
0.00115238
2.85554e-06
9.57811e-10
4.34827e-14
2.67162e-19
2.22152e-25
0
0
max_prob_idx 14, max_prob 0.469448
---------------TIME STEP---------------
t = 14
landmark_positions 3 9 14 23 
observations 7 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			4.96947e-19		0.00013383			6.65065e-23
1			1.75996e-17		1.48672e-06			2.61656e-23
2			2.53232e-16		6.07588e-09			1.53861e-24
3			1.6391e-15		0.241971			3.96613e-16
4			5.83605e-15		0.053991			3.15094e-16
5			1.3265e-13		0.00443185			5.87883e-16
6			1.12366e-11		0.00013383			1.5038e-15
7			3.8331e-10		1.48672e-06			5.69874e-16
8			5.05738e-09		6.07588e-09			3.07281e-17
9			3.02681e-08		0.053991			1.6342e-09
10			7.68458e-07		0.00443185			3.40569e-09
11			6.36034e-05		0.00013383			8.51206e-09
12			0.00214309		1.48672e-06			3.18617e-09
13			0.0274215		6.07588e-09			1.6661e-10
14			0.139054		0.053991			0.00750769
15			0.303548		0.241971			0.0734499
16			0.314969		0.398942			0.125654
17			0.163608		0.241971			0.0395885
18			0.0429613		0.053991			0.00231952
19			0.00580991		0.00443185			2.57486e-05
20			0.000405116		0.00013383			5.42168e-08
21			1.43856e-05		1.48672e-06			2.13873e-11
22			2.63392e-07		6.07588e-09			1.60034e-15
23			2.48655e-09		0			0
24			1.19531e-11		0			0
----------NORMALIZED posteriors---------------
2.67583e-22
1.05275e-22
6.19044e-24
1.59574e-15
1.26775e-15
2.36529e-15
6.05038e-15
2.29283e-15
1.23632e-16
6.57506e-09
1.37025e-08
3.42474e-08
1.28193e-08
6.70339e-10
0.0302065
0.295519
0.505558
0.15928
0.00933237
0.000103597
2.18136e-07
8.60498e-11
6.4388e-15
0
0
max_prob_idx 16, max_prob 0.505558
---------------TIME STEP---------------
t = 15
landmark_positions 3 9 14 23 
observations 6 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			2.15527e-19		0.00443185			9.55184e-22
1			7.24541e-18		0.00013383			9.69654e-22
2			9.20995e-17		1.48672e-06			1.36926e-22
3			4.65924e-16		0.398942			1.85877e-16
4			1.13827e-15		0.241971			2.75427e-16
5			1.16599e-14		0.053991			6.29529e-16
6			9.03446e-13		0.00443185			4.00394e-15
7			3.10282e-11		0.00013383			4.15251e-15
8			4.20603e-10		1.48672e-06			6.25319e-16
9			2.67051e-09		0.241971			6.46185e-10
10			5.45533e-08		0.053991			2.94539e-09
11			4.501e-06		0.00443185			1.99478e-08
12			0.000174193		0.00013383			2.33123e-08
13			0.00300848		1.48672e-06			4.47277e-09
14			0.0255263		0.00443185			0.000113129
15			0.11156		0.053991			0.00602324
16			0.256175		0.241971			0.0619869
17			0.313872		0.398942			0.125217
18			0.204227		0.241971			0.0494169
19			0.0708987		0.053991			0.00382789
20			0.0131794		0.00443185			5.84091e-05
21			0.00130302		0.00013383			1.74384e-07
22			6.90757e-05		1.48672e-06			1.02696e-10
23			1.95976e-06		0			0
24			2.96828e-08		0			0
----------NORMALIZED posteriors---------------
3.87273e-21
3.9314e-21
5.55158e-22
7.53626e-16
1.1167e-15
2.55238e-15
1.62337e-14
1.68361e-14
2.53532e-15
2.61992e-09
1.19419e-08
8.08769e-08
9.45181e-08
1.81345e-08
0.000458673
0.0244208
0.251322
0.507683
0.200358
0.0155199
0.000236816
7.07028e-07
4.16375e-10
0
0
max_prob_idx 17, max_prob 0.507683
---------------TIME STEP---------------
t = 16
landmark_positions 3 9 14 23 
observations 5 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			1.03686e-19		0.053991			5.59809e-21
1			3.49582e-18		0.00443185			1.5493e-20
2			4.60065e-17		0.00013383			6.15706e-21
3			2.56182e-16		0.241971			6.19885e-17
4			7.98898e-16		0.398942			3.18714e-16
5			6.16526e-15		0.241971			1.49181e-15
6			3.75048e-13		0.053991			2.02492e-14
7			1.33414e-11		0.00443185			5.91273e-14
8			2.05357e-10		0.00013383			2.74829e-14
9			1.65282e-09		0.398942			6.59381e-10
10			9.55532e-09		0.241971			2.31211e-09
11			1.29374e-07		0.053991			6.98504e-09
12			5.73689e-06		0.00443185			2.5425e-08
13			0.000167446		0.00013383			2.24094e-08
14			0.00261159		0.00013383			3.49509e-07
15			0.0219381		0.00443185			9.72261e-05
16			0.0989664		0.053991			0.00534329
17			0.239928		0.241971			0.0580555
18			0.313989		0.398942			0.125264
19			0.220221		0.241971			0.0532871
20			0.083257		0.053991			0.00449513
21			0.0169512		0.00443185			7.5125e-05
22			0.00185179		0.00013383			2.47826e-07
23			0.000109309		0			0
24			3.4658e-06		0			0
----------NORMALIZED posteriors---------------
2.26995e-20
6.28218e-20
2.4966e-20
2.51355e-16
1.29234e-15
6.0491e-15
8.21077e-14
2.39753e-13
1.11439e-13
2.6737e-09
9.37527e-09
2.83234e-08
1.03095e-07
9.0867e-08
1.41721e-06
0.000394239
0.0216663
0.235407
0.507926
0.216072
0.0182271
0.000304622
1.0049e-06
0
0
max_prob_idx 18, max_prob 0.507926
---------------TIME STEP---------------
t = 17
landmark_positions 3 9 14 23 
observations 4 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			4.45928e-20		0.241971			1.07902e-20
1			1.32202e-18		0.053991			7.13772e-20
2			2.02681e-17		0.00443185			8.98249e-20
3			1.688e-16		0.053991			9.1137e-18
4			1.15206e-15		0.241971			2.78766e-16
5			1.15829e-14		0.398942			4.62091e-15
6			4.07979e-13		0.241971			9.8719e-14
7			1.32452e-11		0.053991			7.15121e-13
8			1.89993e-10		0.00443185			8.42019e-13
9			1.29271e-09		0.241971			3.12799e-10
10			5.33821e-09		0.398942			2.12964e-09
11			1.81191e-08		0.241971			4.38429e-09
12			1.36253e-07		0.053991			7.35642e-09
13			5.14689e-06		0.00443185			2.28102e-08
14			0.000149974		1.48672e-06			2.22969e-10
15			0.00237736		0.00013383			3.18162e-07
16			0.0203901		0.00443185			9.03658e-05
17			0.0940841		0.053991			0.00507969
18			0.233828		0.241971			0.0565795
19			0.314035		0.398942			0.125282
20			0.226336		0.241971			0.0547668
21			0.088098		0.053991			0.0047565
22			0.0184807		0.00443185			8.19037e-05
23			0.00208413		0			0
24			0.000127143		0			0
----------NORMALIZED posteriors---------------
4.37492e-20
2.89402e-19
3.64199e-19
3.69519e-17
1.13027e-15
1.87357e-14
4.0026e-13
2.89949e-12
3.414e-12
1.26826e-09
8.63471e-09
1.77763e-08
2.98269e-08
9.24851e-08
9.04038e-10
1.29e-06
0.000366392
0.0205958
0.229404
0.507961
0.222054
0.0192854
0.000332082
0
0
max_prob_idx 19, max_prob 0.507961
---------------TIME STEP---------------
t = 18
landmark_positions 3 9 14 23 
observations 3 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			3.45684e-20		0.398942			1.37908e-20
1			4.52486e-19		0.241971			1.09488e-19
2			1.03385e-17		0.053991			5.58188e-19
3			2.11127e-16		0.00443185			9.35683e-19
4			3.47469e-15		0.053991			1.87602e-16
5			4.1849e-14		0.241971			1.01262e-14
6			4.58955e-13		0.398942			1.83096e-13
7			7.85311e-12		0.241971			1.90022e-12
8			1.11247e-10		0.053991			6.00632e-12
9			8.58074e-10		0.053991			4.63282e-11
10			3.70063e-09		0.241971			8.95444e-10
11			1.00779e-08		0.398942			4.0205e-09
12			2.23088e-08		0.241971			5.39807e-09
13			1.2587e-07		0.053991			6.79584e-09
14			4.83927e-06		6.07588e-09			2.94028e-14
15			0.000142854		1.48672e-06			2.12384e-10
16			0.00228616		0.00013383			3.05957e-07
17			0.0197968		0.00443185			8.77363e-05
18			0.0922263		0.053991			0.00497939
19			0.231508		0.241971			0.0560183
20			0.314043		0.398942			0.125285
21			0.22866		0.241971			0.055329
22			0.0899495		0.053991			0.00485646
23			0.0190699		0			0
24			0.00217403		0			0
----------NORMALIZED posteriors---------------
5.59336e-20
4.44071e-19
2.26394e-18
3.79501e-18
7.60888e-16
4.10707e-14
7.42615e-13
7.70706e-12
2.43609e-11
1.87901e-10
3.6318e-09
1.63066e-08
2.18939e-08
2.75631e-08
1.19254e-13
8.614e-10
1.24092e-06
0.000355847
0.0201958
0.227203
0.50814
0.224407
0.0196972
0
0
max_prob_idx 20, max_prob 0.50814
---------------TIME STEP---------------
t = 19
landmark_positions 3 9 14 23 
observations 2 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			4.9439e-20		0.241971			1.19628e-20
1			4.3629e-19		0.398942			1.74055e-19
2			1.09631e-17		0.241971			2.65276e-18
3			3.36024e-16		0.00013383			4.49702e-20
4			6.76363e-15		0.00443185			2.99754e-17
5			8.80553e-14		0.053991			4.75419e-15
6			7.50981e-13		0.241971			1.81715e-13
7			4.82956e-12		0.398942			1.92672e-12
8			3.76068e-11		0.241971			9.09975e-12
9			3.28414e-10		0.00443185			1.45548e-12
10			1.94119e-09		0.053991			1.04807e-10
11			6.74566e-09		0.241971			1.63225e-09
12			1.41846e-08		0.398942			5.65883e-09
13			2.03702e-08		0.241971			4.929e-09
14			1.0177e-07		9.13472e-12			9.2964e-19
15			4.69587e-06		6.07588e-09			2.85315e-14
16			0.000140183		1.48672e-06			2.08412e-10
17			0.00225225		0.00013383			3.0142e-07
18			0.019578		0.00443185			8.67667e-05
19			0.0915517		0.053991			0.00494296
20			0.230705		0.241971			0.0558239
21			0.31415		0.398942			0.125328
22			0.229603		0.241971			0.0555572
23			0.0906026		0			0
24			0.0191645		0			0
----------NORMALIZED posteriors---------------
4.94864e-20
7.20011e-19
1.09737e-17
1.86028e-19
1.23999e-16
1.96666e-14
7.51701e-13
7.97023e-12
3.76429e-11
6.02089e-12
4.33554e-10
6.75212e-09
2.34088e-08
2.03897e-08
3.84564e-18
1.18026e-13
8.62138e-10
1.24688e-06
0.000358927
0.0204475
0.230926
0.518443
0.229823
0
0
max_prob_idx 21, max_prob 0.518443
---------------TIME STEP---------------
t = 20
landmark_positions 3 9 14 23 
observations 1 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			9.98176e-20		0.053991			5.38925e-21
1			8.3774e-19		0.241971			2.02709e-19
2			7.31244e-18		0.398942			2.91724e-18
3			2.11133e-16		1.48672e-06			3.13895e-22
4			5.54868e-15		0.00013383			7.42581e-19
5			8.57662e-14		0.00443185			3.80103e-16
6			7.88405e-13		0.053991			4.25668e-14
7			4.36048e-12		0.241971			1.05511e-12
8			1.56562e-11		0.398942			6.24591e-12
9			7.4939e-11		0.00013383			1.00291e-14
10			5.87878e-10		0.00443185			2.60539e-12
11			3.16453e-09		0.053991			1.70856e-10
12			9.56424e-09		0.241971			2.31427e-09
13			1.59341e-08		0.398942			6.35678e-09
14			1.49957e-08		5.05227e-15			7.57625e-23
15			9.1643e-08		9.13472e-12			8.37133e-19
16			4.74242e-06		6.07588e-09			2.88144e-14
17			0.000141978		1.48672e-06			2.11082e-10
18			0.00228448		0.00013383			3.05733e-07
19			0.0198875		0.00443185			8.81386e-05
20			0.0931314		0.053991			0.00502826
21			0.23495		0.241971			0.0568509
22			0.319422		0.398942			0.127431
23			0.229693		0			0
24			0.0846279		0			0
----------NORMALIZED posteriors---------------
2.84545e-20
1.07027e-18
1.54026e-17
1.65732e-21
3.92073e-18
2.00689e-15
2.24747e-13
5.57083e-12
3.29776e-11
5.29523e-14
1.37561e-11
9.02098e-10
1.2219e-08
3.3563e-08
4.00016e-22
4.41995e-18
1.52136e-13
1.11449e-09
1.61423e-06
0.00046536
0.0265485
0.300165
0.672819
0
0
max_prob_idx 22, max_prob 0.672819
---------------TIME STEP---------------
t = 21
landmark_positions 3 9 14 23 
observations 0 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			1.32953e-19		0.00443185			5.89227e-22
1			1.10686e-18		0.053991			5.97606e-20
2			4.81519e-18		0.241971			1.16513e-18
3			5.40722e-17		6.07588e-09			3.28536e-25
4			1.90371e-15		1.48672e-06			2.83028e-21
5			4.17249e-14		0.00013383			5.58406e-18
6			5.02143e-13		0.00443185			2.22542e-15
7			3.22211e-12		0.053991			1.73965e-13
8			1.04595e-11		0.241971			2.53088e-12
9			2.09549e-11		1.48672e-06			3.11541e-17
10			1.18981e-10		0.00013383			1.59232e-14
11			1.03405e-09		0.00443185			4.58275e-12
12			5.1321e-09		0.053991			2.77087e-10
13			1.3215e-08		0.241971			3.19764e-09
14			1.64007e-08		1.02798e-18			1.68596e-26
15			9.86187e-09		5.05227e-15			4.98248e-23
16			1.1266e-07		9.13472e-12			1.02912e-18
17			6.15332e-06		6.07588e-09			3.73869e-14
18			0.000184347		1.48672e-06			2.74072e-10
19			0.00296696		0.00013383			3.97069e-07
20			0.0257981		0.00443185			0.000114333
21			0.119661		0.053991			0.00646064
22			0.289		0.241971			0.0699296
23			0.342483		0			0
24			0.179126		0			0
----------NORMALIZED posteriors---------------
7.70181e-21
7.81134e-19
1.52295e-17
4.29431e-24
3.69947e-20
7.29895e-17
2.90886e-14
2.2739e-12
3.30813e-11
4.07216e-16
2.08133e-13
5.99013e-11
3.62182e-09
4.17965e-08
2.20372e-25
6.51262e-22
1.34516e-17
4.88686e-13
3.58241e-09
5.19011e-06
0.00149445
0.0844473
0.914053
0
0
max_prob_idx 22, max_prob 0.914053
---------------TIME STEP---------------
t = 22
landmark_positions 3 9 14 23 
observations 25 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			1.11533e-19		0			0
1			1.01465e-18		0			0
2			4.06589e-18		0			0
3			1.40652e-17		0			0
4			4.90092e-16		0			0
5			1.60939e-14		0			0
6			2.7645e-13		0			0
7			2.34807e-12		0			0
8			8.9335e-12		0			0
9			1.4573e-11		0			0
10			3.30572e-11		0			0
11			3.97155e-10		0			0
12			3.1571e-09		0			0
13			1.15729e-08		0			0
14			1.7554e-08		0			0
15			1.03274e-08		0			0
16			5.72646e-09		0			0
17			3.54487e-07		0			0
18			1.95648e-05		0			0
19			0.00057853		0			0
20			0.00897402		0			0
21			0.0703818		0			0
22			0.255226		0			0
23			0.385169		0			0
24			0.22574		0			0
----------NORMALIZED posteriors---------------
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
max_prob_idx 0, max_prob nan
---------------TIME STEP---------------
t = 23
landmark_positions 3 9 14 23 
observations 25 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			nan		0			nan
1			nan		0			nan
2			nan		0			nan
3			nan		0			nan
4			nan		0			nan
5			nan		0			nan
6			nan		0			nan
7			nan		0			nan
8			nan		0			nan
9			nan		0			nan
10			nan		0			nan
11			nan		0			nan
12			nan		0			nan
13			nan		0			nan
14			nan		0			nan
15			nan		0			nan
16			nan		0			nan
17			nan		0			nan
18			nan		0			nan
19			nan		0			nan
20			nan		0			nan
21			nan		0			nan
22			nan		0			nan
23			nan		0			nan
24			nan		0			nan
----------NORMALIZED posteriors---------------
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
max_prob_idx 0, max_prob nan
---------------TIME STEP---------------
t = 24
landmark_positions 3 9 14 23 
observations 25 
pseudo_position-----------Motion----------Observation-----------PRODUCT--
0			nan		0			nan
1			nan		0			nan
2			nan		0			nan
3			nan		0			nan
4			nan		0			nan
5			nan		0			nan
6			nan		0			nan
7			nan		0			nan
8			nan		0			nan
9			nan		0			nan
10			nan		0			nan
11			nan		0			nan
12			nan		0			nan
13			nan		0			nan
14			nan		0			nan
15			nan		0			nan
16			nan		0			nan
17			nan		0			nan
18			nan		0			nan
19			nan		0			nan
20			nan		0			nan
21			nan		0			nan
22			nan		0			nan
23			nan		0			nan
24			nan		0			nan
----------NORMALIZED posteriors---------------
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
nan
max_prob_idx 0, max_prob nan
*/