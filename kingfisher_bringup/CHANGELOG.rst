^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kingfisher_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Remove unused library launch files, add kingfisher_controller detection.
* Remove serial proxying scripts from bringup package.

0.0.1 (2013-09-09)
------------------
* First cut of kingfisher_bringup as a standalone repo.
* Uses robot_upstart.
