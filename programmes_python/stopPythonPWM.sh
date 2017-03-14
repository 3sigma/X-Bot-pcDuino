#!/bin/bash

killall -u root -- python
echo 0 > /sys/class/pwm-sunxi/pwm0/duty_percent
echo 0 > /sys/class/pwm-sunxi/pwm1/duty_percent
