/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   kp = Kpi;
   ki = Kii;
   kd = Kdi;

   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

   if(delta_time == 0){
      delta_time = 1;
   }
   diff_err = (cte - err) / delta_time;

   sum_err += cte;

   err = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control;

   control = -kp * err - ki * sum_err - kd * diff_err;

   std::cout<< "err " << err << ", sum_err " << sum_err << ", diff_err " << diff_err << std::endl;
   std::cout<< "-kp*err " << -kp * err << ", -ki*sum_err " << -ki * sum_err << ", -kd*diff_err " << -kd * diff_err << std::endl;

   if(control > output_lim_max){
      control = output_lim_max;
   }
   else if (control < output_lim_min){
      control = output_lim_min;
   }

   std::cout << "control " << control << ", control_lim " << control << std::endl;

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;
}