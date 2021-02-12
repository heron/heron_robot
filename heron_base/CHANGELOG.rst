^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Replace now-defunct UM6 as default, use CV5-25 instead, as-per RPSW-661 (`#10 <https://github.com/heron/heron_robot/issues/10>`_)
  * Remove all references to the now-defunct UM6, as-per RPSW-661
  * Add the Microstrain CV5 launch configuration as the new default. We're going to move forward and publish the driver off our own build farm, so depending on it shouldn't cause long-term issues (though it won't work right now)
  * Remove the notes to check topics & devices; they are checked & are correct.
* Merge pull request `#9 <https://github.com/heron/heron_robot/issues/9>`_ from heron/mag-arg
  Allow disabling the magnetometer input to the IMU filter
* Add an env var to disable the magnetometer input to the IMU filter, for instances where the IMU does not support that
* Merge pull request `#8 <https://github.com/heron/heron_robot/issues/8>`_ from heron/no_imu_compass
  Remove the imu_compass dependency
* Remove the imu_compass dependency, as it does not appear to be used anywhere
* Make the snmp_lights node executable
* Fix a minor formatting typo
* Fix the rosdep for python-pysnmp so it resolves correctly
* Edited heron_robot to work with uuv_simulator (`#5 <https://github.com/heron/heron_robot/issues/5>`_)
  * Removed wireless watcher and mjpeg server
  * Added node to update lights with WiFi status
  * Commented WiFi lights node
  * Removed unnecessary heron_base files and updated other necessary ones
  * Removed install(PROGRAMS scripts/install...)
  * Updated calibration script, currently uses new calibration loading method
  * Added WiFi watcher back in commit
  * Updated calibrate_compass to possibly use a namespace, fixed base.launch to use (mostly) the basic um6 driver, replaced static transforms with robot_description
  * Reverted calibration duration from 10s to 60s
  * Added copyright notice to SNMP lights script, deleted unused calibration write script
  * Replaced kinetic with ROS_DISTRO env var in nmea install script
  * Almost made base.launch work with namespaces (um6, imu_compass can't)
  * Removed enu dependency from heron_nmea package.xml
  * Renamed WiFi topic for Heron's lights to the correct name
  * If not using snmp_lights.py, use old wireless_watcher
  * Updated snmp_lights.py (output topic name and copyright info)
  * Added wireless watcher back as a run dependency for heron_base
  * Added imu_compass.yaml back into PR
  * Moved remap to MCU node
  * Fixed comments in snmp script and removed unnecessary vars in calibrate_compass
  * Removed duplicate dependency
  * Removed imu_compass
  * Added namespace into um6
  * Further adding of namespaces into heron_base
  * Formatting for base.launch
  * Removing extra space
  * Added compute_calibration script in the CMake install() instruciton
* Added script to monitor WiFi and update LEDs accordingly (`#4 <https://github.com/heron/heron_robot/issues/4>`_)
  Added script to monitor WiFi and update LEDs accordingly
* Updated to use heron_control.
* Contributors: Chris I-B, Chris Iverach-Brereton, Guy Stoppi, Tony Baltovski

0.1.5 (2016-08-03)
------------------
* Add missing install rule for heron_base configs.
* Contributors: Tony Baltovski

* Add missing install rule for heron_base configs.
* Contributors: Tony Baltovski

0.1.4 (2016-07-22)
------------------
* Removed heron_bringup as run dependency from heron_base.
* Contributors: Tony Baltovski

0.1.3 (2016-07-22)
------------------
* Moved ekf and imu_compass config to heron_base which fixes the circular dependency issues.  Removed camera launch and configuration files.
* Contributors: Tony Baltovski

0.1.2 (2016-07-19)
------------------
* Added tf and heron_controller as run dependency for heron_base.
* Added controller and ekf to base.launch.
* Contributors: Tony Baltovski

0.1.1 (2016-07-08)
------------------

0.1.0 (2016-07-06)
------------------
* Added Axis ptz to accessories.
* Updated nmea_topic_driver remap.
* Added env-hooks for mag and controller config.  Renamed folder kingfisher to heron.
* Added heron_base which currently only contains the base launch file and removed the many launch files in heron_bringup. Also, added accessories.launch for optional accessories.
* Contributors: Tony Baltovski
