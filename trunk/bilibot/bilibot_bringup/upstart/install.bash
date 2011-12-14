#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#  or
# sudo ./install.bash usb0
#
# where usb0 is whatever network interface you want to set the robot
# up for.  wlan0 is the default.

interface=wlan0

if [ $# -gt 0 ]; then
    if [ "$1" != "" ]; then
        interface=$1
    fi
fi

echo "Installing using network interface $interface."

sed "s/wlan0/$interface/g" < bilibot-start > /usr/sbin/bilibot-start
chmod +x /usr/sbin/bilibot-start
sed "s/wlan0/$interface/g" < bilibot-stop > /usr/sbin/bilibot-stop
chmod +x /usr/sbin/bilibot-stop
sed "s/wlan0/$interface/g" < bilibot.conf > /etc/init/bilibot.conf

cat 56-ftdi-usb.rules > /etc/udev/rules.d/56-ftdi-usb.rules

# Copy files into /etc/ros/electric/bilibot
mkdir /etc/ros
mkdir /etc/ros/electric
cat bilibot.launch > /etc/ros/electric/bilibot.launch

echo '. /opt/ros/electric/setup.bash; export ROS_PACKAGE_PATH=/home/bilibot/ros:${ROS_PACKAGE_PATH}' > /etc/ros/setup.bash

