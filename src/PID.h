#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
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

  void AddToParameterAtIndex(int index, double amount);
 private:
  double Kp;
  double Ki;
  double Kd;
  double p_err;
  double i_err;
  double d_err;

  std::vector<double> dp;
  int step, param_index;
  int n_ajust;
  int n_eval;
  double tolerance;
  double total_err, best_err;
  bool set_twiddle;
  bool tried_adding, tried_subtracting;
};

#endif  // PID_H