#include "wheel_to_steering/wheel_to_steering.h"

void Wheel2Steering::initialize()
{

}

void Wheel2Steering::terminate()
{

}

/* Constructor */
Wheel2Steering::Wheel2Steering() 

{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
Wheel2Steering::~Wheel2Steering()
{
  /* Currently there is no destructor body generated.*/
}

double Wheel2Steering::ConvertWheelToSteeringAngle(double wheel_angle) 
{
  double a3 = -0.003527; //-0.0009287;
  double a2 = -0.001528; //-0.0009204;
  double a1 = 16.06; //15.17;
  double a0 = 0.0;
  
  double steering_angle = a3 * pow(wheel_angle, 3) + a2 * pow(wheel_angle, 2) + a1 * wheel_angle + a0;

  return steering_angle;
}