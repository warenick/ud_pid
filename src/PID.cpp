#include <limits>
#include <cmath>
#include <iostream>
#include "PID.h"

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  PID::d_err = 0.0;
  PID::i_err = 0.0;
  PID::p_err = 0.0;
  set_twiddle = true;
  double tw_denom = 0.1;
  dp = {tw_denom * Kp, tw_denom * Kd, tw_denom * Ki};
  total_err = 0;
  step = 1;
  param_index = 2;
  best_err = std::numeric_limits<double>::max();
  n_ajust = 100;
  n_eval = 1000;
  tried_adding = false;
  tried_subtracting = false;
  tolerance = 0.02;
}

void PID::UpdateError(double cte)
{
  p_err = step == 1 ? cte : p_err;
  d_err = cte - p_err;
  p_err = cte;
  i_err += cte;
  total_err = (step % (n_ajust + n_eval) > n_ajust) ? total_err + pow(cte, 2) : total_err;
  if (set_twiddle && step % (n_ajust + n_eval) == 0)
  {
    total_err /= n_ajust + n_eval;
    if (total_err < best_err)
    {
      best_err = total_err;
      dp[param_index] = (step != n_ajust + n_eval) ? dp[param_index] * 1.1 : dp[param_index];
      param_index = (param_index + 1) % 3;
      tried_adding = tried_subtracting = false;
    }
    if (!tried_adding && !tried_subtracting)
    {
      AddToParameterAtIndex(param_index, dp[param_index]);
      tried_adding = true;
    }
    else if (tried_adding && !tried_subtracting)
    {
      AddToParameterAtIndex(param_index, -2 * dp[param_index]);
      tried_subtracting = true;
    }
    else
    {
      AddToParameterAtIndex(param_index, dp[param_index]);
      dp[param_index] *= 0.9;
      param_index = (param_index + 1) % 3;
      tried_adding = tried_subtracting = false;
    }
    total_err = 0;
    std::cout << "new parameters" << std::endl;
    std::cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << std::endl;
    if ((dp[0] + dp[1] + dp[2]) < tolerance)
    {
      std::cout << "Tolerance is reached" << std::endl;
      std::cout << "final parameters" << std::endl;
      std::cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << std::endl;
      set_twiddle = false;
    }
  }
  step++;
}

double PID::TotalError()
{
  return -Kp * p_err - Ki * i_err - Kd * d_err; // TODO: Add your total error calc here!
}

void PID::AddToParameterAtIndex(int index, double amount)
{
  Kp = index == 0? Kp + amount: Kp; 
  Kd = index == 1? Kd + amount: Kd;
  Ki = index == 2? Ki + amount: Ki;
}