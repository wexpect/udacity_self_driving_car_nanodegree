#include <cmath>
#include <iostream>
#include <vector>

#include "Dense"

// #include <Eigen/Dense>
// #include <Eigen/Core>

#include "grader.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * TODO: complete this function
 */
vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

    // return {1,2,3,4,5,6};


    double a_0 = start[0];
    double a_1 = start[1];
    double a_2 = start[2] / 2;    

    /* 
    NOTE: can not initialize in this way
      main.cpp:50:5: error: could not convert ‘{{pow(T, (double)3), pow(T, (double)4), pow(T, (double)5)}, {((double)3 * pow(T, (double)2)), ((double)4 * pow(T, (double)3)), ((double)5 * pow(T, (double)4))}, {((double)6 * T), ((double)12 * pow(T, (double)2)), ((double)20 * pow(T, (double)3))}}’ 
      from ‘<brace-enclosed initializer list>’ to ‘Eigen::MatrixXd’ {aka ‘Eigen::Matrix<double, -1, -1>’}
    */
    // Eigen::MatrixXd tf_mat = {
    //     {pow(T, 3),     pow(T, 4),      pow(T, 5)},
    //     {3 * pow(T, 2), 4 * pow(T, 3),  5 * pow(T, 4)},
    //     {6 * T,         12 * pow(T, 2), 20 * pow(T, 3)}
    // };

    Eigen::MatrixXd tf_mat(3, 3);
    tf_mat(0, 0) = pow(T, 3);
    tf_mat(0, 1) = pow(T, 4);
    tf_mat(0, 2) = pow(T, 5);

    tf_mat(1, 0) = 3 * pow(T, 2);
    tf_mat(1, 1) = 4 * pow(T, 3);
    tf_mat(1, 2) = 5 * pow(T, 4);

    tf_mat(2, 0) = 6 * T;
    tf_mat(2, 1) = 12 * pow(T, 2);
    tf_mat(2, 2) = 20 * pow(T, 3);
    

    Eigen::MatrixXd final_C(3, 1);
    final_C(0, 0) = end[0] - ( start[0] + start[1] * T + start[2] * pow(T, 2) / 2 );
    final_C(1, 0) = end[1] - (            start[1]     + start[2] * T );
    final_C(2, 0) = end[2] - (                           start[2] );

    Eigen::MatrixXd a_3_to_a5 = tf_mat.inverse() * final_C;

    return {a_0, a_1, a_2, a_3_to_a5(0, 0), a_3_to_a5(1, 0), a_3_to_a5(2, 0)};

}

int main() {

  // create test cases
  vector< test_case > tc = create_tests();

  bool total_correct = true;

  for(int i = 0; i < tc.size(); ++i) {
    vector<double> jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
    bool correct = close_enough(jmt, answers[i]);
    total_correct &= correct;
  }

  if(!total_correct) {
    std::cout << "Try again!" << std::endl;
  } else {
    std::cout << "Nice work!" << std::endl;
  }

  return 0;
}


/*
poly = [ 0, 10, 0, 0, 0, 0,]
target_poly = [ 0, 10, 0, 0, 0, 0,]

poly = [ 0, 10, 0, -8.88178e-16, -0.625, 0.3125,]
target_poly = [ 0, 10, 0, 0, -0.625, 0.3125,]

poly = [ 5, 10, 1, -3, 0.64, -0.0432,]
target_poly = [ 5, 10, 1, -3, 0.64, -0.0432,]

Nice work!
*/