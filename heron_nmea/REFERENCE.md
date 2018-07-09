# Reference for heron_nmea

## Purpose
The *heron_nmea* package allows users to control and receive data from the Heron in the form of NMEA sentences.

## Sentence Types

### Output
| Code | Data |
| --- | --- |
| CPRBS | GPS Time, Battery Voltage, Battery Voltage, Battery Voltage, 0 |
| CPIMU | GPS Time, X Angular Velocity, Y Angular Velocity, Z Angular Velocity, X Linear Acceleration, Y Linear Acceleration, Z Linear Acceleration |
| CPRCM | GPS Time, Compass ID, Heading, Pitch, Roll, IMU Msg Time |
| CPALT | GPS Time, Lights On(1)/Off(0) |
| CPNVG | GPS Time, GPS Lat, North/South, GPS Long, West/East, 1,0,0, Heading, Roll, Pitch, Odometry Msg Time |
| CPNVR | GPS Time, GPS (East) Velocity, GPS (North) Velocity, 0, Y Angular Velocity, X Angular Velocity, Z Angular Velocity |
| CPTIME | GPS Time |

There are also a number of NMEA sentences published directly from the GPS sensor. The sentence type is prefixed by "GP" and can be decoded using this guide: http://www.gpsinformation.org/dale/nmea.htm

#### Details of Output Data
- CPIMU:
  - Y Angular Velocity and Z Angular Velocity are negative compared to the direct output from the IMU (in degrees)
  - Similarly for Y,Z linear acceleration
- CPRCM:
  - Heading field is (180 - deg(h)) where h is the heading from the IMU
  - Pitch is negative compared to the output from the IMU
  - All fields are in degrees
- CPNVG:
  - Latitude and Longitude in Degrees and Decimal Minutes (DMM)
  - Heading, Pitch, Roll are in degrees
- CPNVR:
  - The Y,Z angular velocity are negative compared to the direct output of the IMU
  - Angular velocities are in degrees

### Input
| Code | Published Topic | Description |
| --- | --- | --- |
| PYDIR | cmd_drive | Left Motor Power, Right Motor Power |
| PYDEV | cmd_course |  Desired Yaw, Desired Speed |
| PYDEP | cmd_helm | Desired Yaw Rate, Thrust |
| PYCLT | disable_lights | 1 or 0 (on or off resp.) |

#### Details of Input Data
- PYDIR:
  - Range for motor powers is -100 to 100 (representing percentage)
- PYDEV:
  - Published yaw is ```to_radians(90 - y)``` where y is the value given in the input sentence
- PYDEP:
  - Published yaw rate is ```-1 * to_radians(y)``` where y is the value given in the input sentence
  - Range of possible thrust values is -100 to 100 (representing percentage)

## nmea_if.launch
The NMEA interface is launched using ```heron_nmea/launch/nmea_if.launch```

This file launches the following nodes:  

| Node Type | Package | Description |
| --- | --- | --- |
| Relay | topic_tools | Relays GPS NMEA sentences to *nmea_if/tx* topic |
| gps_time_offset_publisher | heron_nmea | |
| throttle | topic_tools | Throttles *imu/data* topic |
| throttle | topic_tools | Throttles *sense* topic
| imu_publisher | heron_nmea | Publishes CPIMU,CPRCM sentences |
| throttle | topic_tools | Throttles *lights* topic |
| lights_publisher | heron_nmea | Publishes CPALT sentences |
| command_publisher | heron_nmea | Listens to PYDIR,PYDEV,PYDEP,PYCLT and publishes on the correct rostopics |
| socket_node | nmea_comms | Subscribes to input and publishes output NMEA sentences |
