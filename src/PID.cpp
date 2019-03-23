#include "PID.h"
#include <cmath>
#include <iostream>
#include<vector>
using std::vector;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  prev_cte = 0.0;  

}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
  
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  double tot_err = -p_error*Kp - i_error*Ki - d_error*Kd;
  if (tot_err > 1) {
    tot_err = 1.0;
  }else if (tot_err < -1) {
    tot_err = -1.0;
  }
  
  return tot_err;  // Add your total error calc here!
  
}

void PID::twiddle(double tol) {
  vector<double> p = {Kp, Ki, Kd};
  vector<double> dp = {0.1,0.1,0.1};
  double sum = 0.0;
  for (int i =0; i < p.size(); i++) {
    sum += dp[i];
  }
  //while ()

  
}

