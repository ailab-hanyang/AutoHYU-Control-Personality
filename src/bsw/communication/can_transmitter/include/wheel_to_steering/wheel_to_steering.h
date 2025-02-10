#ifndef WHEEL_TO_STEERING_h_
#define WHEEL_TO_STEERING_h_
#include <math.h>


class Wheel2Steering
{
  /* public data and function members */
 public:

  void initialize();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  Wheel2Steering();

  /* Destructor */
  ~Wheel2Steering();

  double ConvertWheelToSteeringAngle(double wheel_angle);

};


#endif  /* WHEEL_TO_STEERING_h_ */
