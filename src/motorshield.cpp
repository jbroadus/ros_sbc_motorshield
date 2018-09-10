#include <pigpio.h>

#include "ros_sbc_motorshield/dc_motor.h"


int main(int argc, char **argv)
{
  if (gpioInitialise() < 0) {
    return 1;
  }
  ros::init(argc, argv, "motorshield");
  Motor m0(0);
  Motor m1(1);
  Motor m2(2);
  Motor m3(3);

  ros::spin();

  gpioTerminate();

  return 0;
}

