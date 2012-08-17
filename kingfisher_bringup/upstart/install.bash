#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#  or
# sudo ./install.bash usb0
#  or
# sudo ./install.bash usb0 fuerte
#
# where usb0 is whatever network interface you want to set the robot
# up for.
# and fuerte is the specified version of ROS to use.
# Default is the latest installed.

interface=$(iwconfig 2>/dev/null | awk '{print $1}' | head -n1)

#stackPath=/opt/ros/fuerte/stacks/kingfisher/kingfisher_bringup/upstart
stackPath=./

if [ $# -gt 0 ]; then
    if [ "$1" != "" ]; then
        interface=$1
    fi
fi

release=$(ls /opt/ros/ | tail -n1)

if [ $# -gt 1 ]; then
    if [ "$2" != "" ]; then
        release=$2
    fi
fi


source /opt/ros/$release/setup.bash
OLD_DIR=$(pwd)
cd `rospack find kingfisher_bringup`/upstart

# checks if kingfisher user+group exists, if it doesn't, then it creates a kingfisher daemon.

if ! grep "^kingfisher:" /etc/group >/dev/null 2>&1; then
    echo "Group kingfisher does not exist, creating."
    groupadd kingfisher
fi

if ! id -u kingfisher >/dev/null 2>&1; then
    echo "User kingfisher does not exist, creating and adding it to groups kingfisher and sudo."
    useradd -g kingfisher kingfisher
    usermod kingfisher -G sudo
    if [ ! -e /home/kingfisher ]; then
        echo "Turtlebot home directory was not created, creating."
        mkdir /home/kingfisher
        chown kingfisher:kingfisher /home/kingfisher
    fi
fi

cp $stackPath/52-kingfisher.rules /etc/udev/rules.d/

source /opt/ros/$release/setup.bash

echo "Installing using network interface $interface."

sed "s/wlan0/$interface/g" < $stackPath/kingfisher-start | sed "s/release/$release/"g > /usr/sbin/kingfisher-start
chmod +x /usr/sbin/kingfisher-start
sed "s/wlan0/$interface/g" < $stackPath/kingfisher-stop | sed "s/release/$release/"g > /usr/sbin/kingfisher-stop
chmod +x /usr/sbin/kingfisher-stop
sed "s/wlan0/$interface/g" < $stackPath/kingfisher.conf > /etc/init/kingfisher.conf

# Copy files into /etc/ros/$release/kingfisher
mkdir -p /etc/ros
mkdir -p /etc/ros/$release
cat $stackPath/kingfisher.launch > /etc/ros/$release/kingfisher.launch

echo ". /opt/ros/$release/setup.bash;" > /etc/ros/setup.bash

cd $OLD_DIR
