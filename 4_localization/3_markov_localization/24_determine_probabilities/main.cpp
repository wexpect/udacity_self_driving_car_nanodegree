#include <iostream>
#include "help_functions.h"

// TODO: assign a value, the difference in distances between x_t and x_{t-1}
// that maximizes the returned probability of norm_pdf

// YOUR VALUE HERE
float value = 1;  // output is prob = 0.398942
// float value = 10;  

float parameter = 1.0;  // set as control parameter or observation measurement
float stdev = 1.0;  // position or observation standard deviation

int main() {

    float prob = Helpers::normpdf(value, parameter, stdev);
    std::cout << prob << std::endl;

    return 0;
}