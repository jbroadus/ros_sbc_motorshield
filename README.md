# ros_sbc_motorshield
ROS package for SB Components' Raspberry Pi motor shield.

SB Component's Python library used for reference:
https://github.com/sbcshop/MotorShield

Initially, I've only added support for DC motors. This version depends on pigpio which requires write access to /dev/mem. I plan on exploring wiringpi or other options in the near future to avoid running as root.
