#ifndef PID_H
#define PID_H

#include <limits>
#include <iostream>
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Use Twiddle algorithm to tune PID parameters given cross track error.
   * @param cte The current cross track error
   */
  void Twiddle(double cte);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double total_error;
  double steer_value;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Twiddle variables
   */
  double p[3];
  double dp[3];
  int twiddle_step;
  int index;
  double best_err;
  double tolerance;
  /** 
   * Initialization variable
   */
  bool init;
};

#endif  // PID_H