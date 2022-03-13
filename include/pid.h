#ifndef _PID_H_
#define _PID_H_

#include <vector>
#include <cstdint>

class PID{
  public:
    PID(double Kp, double Ki, double Kd, uint8_t max_window, double dt);
    double get_P_value(double err);
    double get_I_value(double err);
    double get_D_value(double err);

  private:
    double Kp;
    double Ki;
    double Kd;
    double dt;
    uint32_t max_window;
    uint32_t err_idx;
    std::vector<double> err_list;
};

#endif