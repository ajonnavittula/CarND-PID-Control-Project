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

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

}

double PID::TotalError(double cte) {
  /**
   * TODO: Calculate and return the total error
   */
  i_error += cte;
  return i_error;  // TODO: Add your total error calc here!
}