^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_nmea
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
