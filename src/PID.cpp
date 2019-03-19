#include "PID.h"
#include <iterator>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  p[0] = Kp_;
  p[1] = Ki_;
  p[2] = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  num_steps = 0;

  resetDp();
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

    // limit accumulation to I_ERR_MAX_STEPS number of steps
    if (num_steps % I_ERR_MAX_STEPS == 0)
        i_error = 0;
    else
        i_error += cte;

    d_error = cte - p_error;
    p_error = cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return p[0]*p_error + p[1]*i_error + p[2]*d_error;  // TODO: Add your total error calc here!
}

void PID::resetDp() {
    for (size_t k=0; k < 3; ++k) {
        dp[k] = 0.001;
    }
}

void PID::twiddleIfNecessary(double cte) {
    num_steps++;
    if (num_steps % STEPS_PER_CYCLE != 0) {
        if (num_steps % STEPS_PER_CYCLE == 1)
            err = 0;

        err += cte * cte;
    }
    else {
        err /= STEPS_PER_CYCLE;
        //Try to twiddle the coeffs when the ave. error is > MAX_ALLOWED_CTE
        if (err > MAX_ALLOWED_CTE) {
            if (goto_label == "STEP_1")
                goto STEP_1;
            if (goto_label == "STEP_2")
                goto STEP_2;
            if (goto_label == "STEP_3")
                goto STEP_3;

            //obtain best_error (1st cycle)
            best_err = err;
            idx = 0;
            goto_label = "STEP_1";
            return;

            STEP_1:
            std::cout << "TWIDDLE_1" << std::endl;
            //std::cout << "idx: "<< idx << ", STEP_1 err: " << err << ", " << best_err << std::endl;
            //try to add twiddle to the currently adjusted coeff (2nd cycle)
            p[idx] += dp[idx];
            goto_label = "STEP_2";
            return;

            STEP_2:
            std::cout << "TWIDDLE_2" << std::endl;
            //std::cout << "idx: "<< idx << ", STEP_2 err: " << err << ", " << best_err <<", i_err: " << i_error << std::endl;

            // if 1st cycle did not decrease the error, try to subtract twiddle
            // from the currently adjusted coeff (3rd cycle)
            if (err < best_err) {
                best_err = err;
                dp[idx] *= 1.1;
            }
            else {
                p[idx] -= 2*dp[idx];
                goto_label = "STEP_3";
                return;

                STEP_3:
                std::cout << "TWIDDLE_3" << std::endl;
                //std::cout << "idx: "<< idx << ", STEP_3 err: " << err << ", " << best_err << std::endl;

                // if 2nd cycle did not decrease the error, try to subtract twiddle
                // from the currently adjusted coeff (4th cycle)
                if (err < best_err) {
                    best_err = err;
                    dp[idx] *= 1.1;
                }
                else {
                    p[idx]  += dp[idx];
                    dp[idx] *= 0.9;
                }
            }

            ++idx %= 3;
            goto_label = "STEP_1";
        }
        else {
            std::cout << "RESET" << std::endl;
            best_err = 0;
            i_error = 0;
            goto_label = "";
            resetDp();
            idx = 0;
        }
    }
}
