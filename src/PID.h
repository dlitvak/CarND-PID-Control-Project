#ifndef PID_H
#define PID_H

#include <string>

using std::string;

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

// private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */

  // number of measurements from Simulator
  int num_steps;

  const int STEPS_PER_CYCLE = 5;

  //accum cycle error
  double err = 0;

  const double MAX_ALLOWED_CTE = 0.5;

  double best_err = 0;
  string goto_label = "";
  double dp[3];
  double p[3] = {0.06, 0.0, 0.0};
  int idx = -1;

  const int I_ERR_MAX_STEPS = 50;

  void resetDp();

    void twiddleIfNecessary(double d);
};

#endif  // PID_H