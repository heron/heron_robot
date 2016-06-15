^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2015-04-14)
------------------
* Add echosounder nmea topic
* Changed sensor to sonar to be more descriptive.
* set ublox back as default. Minor syntax changes.
* Setup sonar, gps, rtcm
* changes to nmea gps and serial bringup and added rtcm netserial stream
* Contributors: Andrew Blakey, Graeme Yeates, Mike Purvis, andrew_blakey

0.0.4 (2014-03-19)
------------------
* Install calibrate_compass script.
* Switch to internal robot_pose_ekf fork (for now)
* Add description.launch to basic install
* Add robot_pose_ekf to default launchers.
* Add wireless_watcher dependency.
* Contributors: Mike Purvis

0.0.2 (2013-10-24)
------------------
* New default compass config from CAN22, plus message about running the calibration.
* rosbag in Hydro can't seem to be kill'd properly, so switch to using the --duration parameter.
* Add calibrate-compass script from 2013-07 shipment.
* Eliminate separate declination apply node.
* Respawn the wireless watcher; had an issue with it dying.
* Fixes to namespaces and remapping to get things talking properly.
* Add the controller launch file to bringup package, at least for now.
* Respect the ROBOT_NETWORK and ROBOT_SETUP env vars.
* Add the imu and controller config files.
* Remove unused library launch files, add heron_controller detection.
* Remove serial proxying scripts from bringup package.

0.0.1 (2013-09-09)
------------------
* First cut of kingfisher_bringup as a standalone repo.
* Uses robot_upstart.
