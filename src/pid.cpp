#include "pid.h"

PID::PID(double Kp, double Ki, double Kd, uint8_t max_window, double dt):
  Kp{Kp},
  Ki{Ki},
  Kd{Kd},
  dt{dt},
  max_window{max_window},
  err_idx{0},
  err_list(max_window, 0.0) {}

double PID::get_P_value(double err){
  return this->Kp*err;
}

double PID::get_I_value(double err){
  this->err_list.at((int)((this->err_idx)%(this->max_window))) = err;

  double sum = 0;
  for(double i : this->err_list) sum += i;

  return this->Ki*sum*dt;
}

double PID::get_D_value(double err){
  // I mean... PI is pretty good :V
  return 0.0;
}