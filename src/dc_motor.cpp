#include <pigpio.h>
#include <string>

#include "ros_sbc_motorshield/dc_motor.h"

#define NUM_CONFIGS 2
#define NUM_MOTORS 4


MotorPins config_pins[NUM_MOTORS][NUM_CONFIGS] =
{
  /* Motor 1 */
  {
    { 17, 22, 27 },
    { 17, 27, 22 }
  },
  /* Motor 2 */
  {
    { 25, 23, 24 },
    { 25, 24, 23 }
  },
  /* Motor 3 */
  {
    { 10, 9, 11 },
    { 10, 11, 9 }
  },
  /* Motor 4 */
  {
    { 12, 8, 7 },
    { 12, 7, 8 }
  }
};

Motor::Motor(unsigned int num) :
  nh(std::string("dc_motor") + std::to_string(num))
{
  motor_num = num;
  motor_config = 0;
  motor_speed = 0;
  pins = NULL;
  sub_speed = nh.subscribe("speed", 100, &Motor::set_speed, this);
  sub_config = nh.subscribe("config", 100, &Motor::set_config, this);
  update_pins();
}

void Motor::set_speed(const std_msgs::Int32::ConstPtr& msg)
{
  motor_speed=msg->data;
  ROS_INFO("Speed: %d", motor_speed);
  if (valid_config())
    update_speed();
}

void Motor::set_config(const std_msgs::UInt8::ConstPtr& msg)
{
  motor_config = msg->data;
  
  ROS_INFO("Config: %d", motor_config);
  update_pins();
  if (valid_config()) {
    update_speed();
  }
}

void Motor::set_pwm(unsigned int speed)
{
  unsigned int pwm_duty = speed;
  if (pwm_duty > pwm_range)
    pwm_duty = pwm_range;
  ROS_INFO("Setting PWM %d to %u\n", pins->pwm, pwm_duty);
  gpioPWM(pins->pwm, pwm_duty);
}

/* Expects pins to be non-NULL */
void Motor::update_speed()
{
  if (motor_speed > 0) {
    /* Forwards */
    gpioWrite(pins->backward, 0);
    set_pwm(motor_speed);
    gpioWrite(pins->forward, 1);
  }
  else if (motor_speed < 0) {
    /* Backwards */
    gpioWrite(pins->forward, 0);
    set_pwm(-motor_speed);
    gpioWrite(pins->backward, 1);
  }
  else {
    set_pwm(0);
    gpioWrite(pins->backward, 0);
    gpioWrite(pins->forward, 0);
  }
}

void Motor::update_pins()
{
  if (motor_num >= NUM_MOTORS) {
    ROS_ERROR("Illegal motor: %u\n", motor_num);
    pins = NULL;
    return;
  }

  if (motor_config >= NUM_CONFIGS) {
    ROS_ERROR("Illegal config option: %u\n", motor_config);
    pins = NULL;
    return;
  }

  ROS_INFO("Configuring motor %u with config %u\n", motor_num, motor_config);
  pins = &config_pins[motor_num][motor_config];

  gpioSetMode(pins->pwm, PI_OUTPUT);
  gpioSetMode(pins->forward, PI_OUTPUT);
  gpioSetMode(pins->backward, PI_OUTPUT);
  pwm_range = gpioGetPWMrange(pins->pwm);
}
