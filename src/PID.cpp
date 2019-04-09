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
  Kd = Kd_;

  p_error = 0.;
  i_error = 0.;
  d_error = 0.;

  init = false;

  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
  dp[0] = 0.01;
  dp[1] = 0.00001;
  dp[2] = 0.001;
  twiddle_step = 0;
  index = 0;
  best_err = std::numeric_limits<double>::max();
  tolerance = 0.002;
  time_step = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  if ( !init ) { 
    p_error = cte;
    init = true;
  }

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

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

if (time_step < 100000 && (time_step % 100 == 0))
  {
    //if (best_err > tolerance) {

      switch(twiddle_step) {

        case 0: {
                  p[index] += dp[index];
                  twiddle_step++;
                  std::cout<< "In Twiddle case 0" << std::endl;
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
                  std::cout<< "In Twiddle case 1" << std::endl;
                  break;
                }
        case 2: {
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
                  std::cout<< "In Twiddle case 2" << std::endl;
                  break;
                }
        default:
                std::cout<< "Unknown twiddle step reached. Twiddle Step: "
                          << twiddle_step;
                break;
      }
    //}
  }
  time_step++;
}

double PID::GetGain(int i) {

  switch(i) {

    case 0: return Kp;
    case 1: return Ki;
    case 2: return Kd;
    default: return 0; 
  }
}

