#include "motion.h"

Motion::Motion(motor& l, motor& r, uint8_t mv, double wd, uint16_t tw):
  max_vel{mv},
  left_m{l},
  right_m{r},
  dTrain{drivetrain(l, r, 3.1416*wd, tw, 130, mm)}
{}

void Motion::move_with_vel(int8_t linear, int8_t angular){
  int8_t l_vel = linear + angular;
  int8_t r_vel = linear - angular;

  /*
    TODO: if both l_vel and r_vel > max_vel, scale them down
    while maintaining their current ratio? idk
  */
  if (l_vel > this->max_vel) l_vel = this->max_vel;
  if (r_vel > this->max_vel) r_vel = this->max_vel;

  this->left_m.setVelocity(l_vel, percent);
  this->right_m.setVelocity(r_vel, percent);

  this->left_m.spin(forward);
  this->right_m.spin(forward);
}

void Motion::move_for_dist(uint16_t dist, uint8_t vel){
  this->dTrain.setDriveVelocity(vel, percent);
  this->dTrain.driveFor((double)dist, mm);
}

void Motion::turn_for_angle(int16_t a, int8_t vel){
  this->dTrain.setTurnVelocity(vel, percent);
  this->dTrain.turnFor((double)a, degrees);
}

void Motion::stop(){
  this->left_m.setVelocity(0, percent);
  this->right_m.setVelocity(0, percent);
}

bool Motion::move_to_object(vision& v_sensor, signature obj, double kvp, double kvi,
                        double kap, double kai, double max_v, double max_a, double window,
                        double close_dist, double rate)
{
  PID pid_v(kvp, kvi, 0, window, 1.0/rate);
  PID pid_a(kap, kai, 0, window, 1.0/rate);

  // if(v_sensor.objectCount == 0) return false;

  uint16_t cx;
  uint8_t stop_ctr = 0;

  while(v_sensor.objects[0].width < close_dist+10){
    v_sensor.takeSnapshot(obj);
    cx = v_sensor.objects[0].centerX;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
  
    Brain.Screen.print(cx);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print(Vision.objects[0].width);

    // PID angular velocity
    int16_t ang_err = cx - 157;
    double ang_vel = pid_a.get_P_value(ang_err) + pid_a.get_I_value(ang_err);

    if(ang_vel > max_a) ang_vel = max_a;
    if(ang_vel < -max_a) ang_vel = -max_a;


    // PID linear velocity
    int16_t dist_err = 220 - Vision.objects[0].width;
    double vel = pid_v.get_P_value(dist_err) + pid_v.get_I_value(dist_err);

    if(vel > max_v) vel = max_v;
    if(vel < -max_v) vel = -max_v;

    this->move_with_vel(vel, -ang_vel);

    if (dist_err < 3) stop_ctr++;
    if (stop_ctr > 10) break;
    wait(1.0/rate, seconds);
  }

  return true;
}
