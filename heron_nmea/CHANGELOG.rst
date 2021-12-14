^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_nmea
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2021-12-14)
------------------

0.1.7 (2021-05-18)
------------------

0.1.6 (2021-02-12)
------------------
* [heron_nmea] Switched to std_srvs. (`#7 <https://github.com/heron/heron_robot/issues/7>`_)
* Update to NMEA package to work with the other new packages (`#6 <https://github.com/heron/heron_robot/issues/6>`_)
  * Added possibility of custom namespace into heron_nmea
  * Removed outdated test launch files
  * First step of adding namespaces into heron_nmea
  * Removing unnecessary topic_tools relays
  * Changed NVG position info from EKF algorithm to GPS
  * Had command publisher turn on/off the heron_controller as needed
  * Added nav_msgs as run dependency for nav_publisher
  * Adding namespaces into heron_nmea package
  * Fixed script permissions and service calls
  * Proper parameter loading, permissions
  * Added reference guide for heron_nmea package
  * Further updates to NMEA reference file
  * Updated heron_nmea Reference.MD
  * Changes that were removed for some reason
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
* Added vessel light off/on control.
* Added Ceepulse sonar accessory.
* Contributors: Guy Stoppi, Tony Baltovski

0.1.5 (2016-08-03)
------------------
* Updated the remap for the nmea_if socket node.
* Contributors: Tony Baltovski

0.1.4 (2016-07-22)
------------------

0.1.3 (2016-07-22)
------------------

0.1.2 (2016-07-19)
------------------
* Set queue_size to python publishers.
* Added navsat_sentence_relay.
* Contributors: Tony Baltovski

0.1.1 (2016-07-08)
------------------

0.1.0 (2016-07-06)
------------------
* Added env-hooks for mag and controller config.  Renamed folder kingfisher to heron.
* Added dependency on heron_msgs_gencpp.  Switch to system headers and added boost to CMakeLists.txt.
* Heron rename.
* Contributors: Tony Baltovski

0.0.5 (2015-04-14)
------------------
* Instantiate Helm and Course classes as well as Drive.
* Fix topic name for Hydro, will change back on Indigo.
* Remove screen output tags from launch file.
* Add C++ implementation of command_publisher.
* changes to nmea gps and serial bringup and added rtcm netserial stream
* Contributors: Andrew Blakey, Mike Purvis

0.0.4 (2014-03-19)
------------------
* Fixes for NMEA interface spec.
* Catkin fixes to the nmea python scripts.
* Add mjpeg_server dependency
* Contributors: Mike Purvis

0.0.3 (2014-01-30)
------------------
* Catkinization of kingfisher_nmea; still have a service call problem with ENU to resolve.

0.0.2 (2013-10-24)
------------------
* Move kingfisher_nmea to subdirectory.

0.0.1 (2013-09-09)
------------------
