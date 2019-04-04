#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd_ = Kd_;

  p_error = 0.;
  i_error = 0.;
  d_error = 0.;

  init = false;

  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
  dp[0] = 0.01;
  dp[1] = 0.0001;
  dp[2] = 0.1;
  twiddle_step = 0;
  index = 0;
  best_err = std::numeric_limits<double>::max();
  tolerance = 0.002;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  if ( !init) { 
    p_error = cte;
    init = true;
  }

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  Kp = p[0];
  Kd = p[1];
  Ki = p[2];

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  total_error = (-Kp * p_error - Kd * d_error - Ki * i_error);
  
  if (total_error < -1) { total_error = -1; }
  else if (total_error > 1) { total_error = 1; }

  return total_error;
}

void PID::Twiddle(double cte) {


  if (best_err > tolerance) {

    switch(twiddle_step) {

      case 0: {
                p[index] += dp[index];
                twiddle_step++;
                break;
              }
      case 1: {
                if (cte < best_err) {
                  best_err = cte;
                  dp[index] *= 1.1;
                }
                else {
                  p[index] -= 2 * dp[index];
                }              
                twiddle_step++;
                break;
              }
      case 3: {
                if (cte < best_err) {
                  best_err = cte;
                }
                else {
                  p[index] += dp[index];
                  dp[index] *= 0.9;
                }
                twiddle_step = 0;
                if (index < 2) { index++; }
                else {index = 0;}
                break;
              }
      default:
              std::cout<< "Unknown twiddle step reached. Twiddle Step: "
                        << twiddle_step;
              break;
    }
  }
}
