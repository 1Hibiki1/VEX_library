#ifndef _MOTION_H_
#define _MOTION_H_

#include <cstdint>
#include <iostream>
#include "vex.h"
#include "pid.h"

class Motion{
  public:
    Motion(motor& left, motor& right, uint8_t max_vel,
           double wheel_diameter, uint16_t track_width);
    void move_with_vel(int8_t, int8_t);
    void move_for_dist(uint16_t dist, uint8_t vel);
    void turn_for_angle(int16_t angle, int8_t vel);
    bool move_to_object(vision&, signature obj, double kvp, double kvi,
                        double kap, double kai, double max_v, double max_a, double window,
                        double close_dist, double rate);
    void stop();

  private:
    uint8_t max_vel;

    motor& left_m;
    motor& right_m;
    drivetrain dTrain;
};

#endif