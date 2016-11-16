^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2016-08-03)
------------------
* Added calibrate_compass and netserial scripts in heron_bringup to be installed.
* Contributors: Tony Baltovski

0.1.4 (2016-07-22)
------------------

0.1.3 (2016-07-22)
------------------
* Moved ekf and imu_compass config to heron_base which fixes the circular dependency issues.  Removed camera launch and configuration files.
* Contributors: Tony Baltovski

0.1.2 (2016-07-19)
------------------
* Added controller and ekf to base.launch.
* Added heron_base as run dependency to heron_bringup.
* Contributors: Tony Baltovski

0.1.1 (2016-07-08)
------------------
* Fixed axis ptz comment.
* Updated Axis PTZ default width and height.
* Contributors: Tony Baltovski

0.1.0 (2016-07-06)
------------------
* Added Axis ptz to accessories.
* Updated nmea_topic_driver remap.
* Added lms1xx accessory and some missing run dependencies.
* Added env-hooks for mag and controller config.  Renamed folder kingfisher to heron.
* Added RTK relay from Jackal.
* Added heron_base which currently only contains the base launch file and removed the many launch files in heron_bringup. Also, added accessories.launch for optional accessories.
* Split the old heron_bring/install script into heron_bringup/install script which installs the robot_upstart job and heron_bringup/setup script which performs the configuration for udev rules as well as the ublox.
* Heron rename.
* Contributors: Tony Baltovski

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
