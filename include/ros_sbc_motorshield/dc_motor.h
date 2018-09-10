#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt8.h"

struct MotorPins
{
  unsigned int pwm;
  unsigned int forward;
  unsigned int backward;
};

class Motor
{
public:
  Motor(unsigned int num);

 protected:
  unsigned int motor_num;
  unsigned int motor_config;
  unsigned int pwm_range;
  int motor_speed;
  ros::NodeHandle nh;
  ros::Subscriber sub_speed, sub_config;
  MotorPins *pins;

  void set_speed(const std_msgs::Int32::ConstPtr& msg);
  void set_config(const std_msgs::UInt8::ConstPtr& msg);

  void update_config();
  void update_speed();
  void update_pins();
  void set_pwm(unsigned int speed);

  bool valid_config() { return pins != NULL; }
};
