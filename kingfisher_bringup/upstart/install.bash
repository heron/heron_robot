#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#

stackPath=./

robot=kingfisher
release=$(ls /opt/ros/ | tail -n1)

source /opt/ros/$release/setup.bash
pushd `rospack find kingfisher_bringup`/upstart > /dev/null

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
        echo "Kingfisher home directory was not created, creating."
        mkdir /home/kingfisher
        chown kingfisher:kingfisher /home/kingfisher
    fi
fi

cp `rospack find ${robot}_bringup`/udev/* /etc/udev/rules.d/

source /opt/ros/$release/setup.bash

function do_subs {
  # source file, dest file, interface, robot, job, release
  cp $1 $2
  sed -i "s/interface0/$3/g" $2
  sed -i "s/portnum/$4/g" $2
  sed -i "s/robot/$5/g" $2
  sed -i "s/job/$6/g" $2
  sed -i "s/release/$7/g" $2 
}

function install_job {
  job=$1
  interface=$2
  portnum=$3

  echo "Installing $robot-$job using network interface $interface, port $portnum."
  
  cp $stackPath/mklaunch /usr/sbin/mklaunch
  do_subs $stackPath/start /usr/sbin/$robot-$job-start $interface $portnum $robot $job $release
  chmod +x /usr/sbin/$robot-$job-start

  do_subs $stackPath/stop /usr/sbin/$robot-$job-stop $interface $portnum $robot $job $release
  chmod +x /usr/sbin/$robot-$job-stop

  do_subs $stackPath/job.conf /etc/init/$robot-$job.conf $interface $portnum $robot $job $release

  # Copy launch files into /etc/ros/
  launch_path=/etc/ros/$release/$robot/$job.d
  mkdir -p $launch_path 
  cp `rospack find ${robot}_bringup`/launch/$job/* $launch_path
}

# substitutions: interface0, robot, job, release
install_job core eth1 11310
install_job interface wlan0 11311

echo ". /opt/ros/$release/setup.bash;" > /etc/ros/setup.bash

popd > /dev/null
