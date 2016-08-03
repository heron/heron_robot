^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
