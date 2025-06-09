**This ROS node reads NMEA sentences from a GNSS receiver via serial port, parses key data using pynmea2, and publishes the information on standard ROS topics. It supports position, velocity, heading, GNSS quality, and GNSS time reference outputs.**

**Purpose**
To convert raw NMEA GNSS data into standard ROS messages such as:

GPS fix (/gps/fix)

Heading (/gnss/hdt/heading)

Speed and course (/gnss/gnvtg/speed_course)

GNSS time (/gnss/time_reference)

GNSS diagnostics and info strings

**How does it work?**

'''roslaunch gnss_parser gnss_parser.launch'''

This launch file will:

1.  Start the gnss_nmea_parser.py node from the gnss_parser package

2.  Connect to /dev/ttyUSB1 at 115200 baud

3.  Output parsed GNSS data to standard ROS topics

✅ You can modify the serial_port or baud_rate values as needed in the launch file or override them via command line arguments.
Opens a serial connection (default: /dev/ttyUSB1) to the GNSS module.

Parses specific NMEA sentences (GNGGA, GNRMC, GNVTG, GNGSA, GNGLL, HDT).

Publishes parsed values to relevant ROS topics.


**Published ROS Topics**
**Topic Name	**                     ** Message Type**	                                   ** Description**
/gps/fix	                    sensor_msgs/NavSatFix	                    GPS position (latitude, longitude, altitude)
/gnss/gngga/quality	          std_msgs/String	                          GNSS fix quality, satellite count, HDOP, and altitude from GGA
/gnss/gnrmc/info	            std_msgs/String	                          UTC time, speed over ground, course, date from RMC
/gnss/gnvtg/speed_course	    geometry_msgs/Twist	                      Speed (linear.x) in km/h and course angle (angular.z) from VTG
/gnss/gngsa/dop	              std_msgs/String	                          Fix type and DOP values (PDOP, HDOP, VDOP) from GSA
/gnss/gngll/position	        std_msgs/String	                          Latitude and longitude string with timestamp from GLL
/gnss/hdt/heading	            geometry_msgs/QuaternionStamped	          True heading converted to quaternion (used for orientation)
/gnss/time_reference	        sensor_msgs/TimeReference	                GNSS-provided UTC time as a ROS time reference

**Supported NMEA Sentences**
**Sentence Type	**   ** Description**
GNGGA	            GNSS Fix Data
GNRMC	            Recommended Minimum GNSS Data
GNVTG	            Course Over Ground & Speed
GNGSA	            GNSS DOP and Active Satellites
GNGLL	            Geographic Position – Latitude/Longitude
HDT	              True Heading (manual parsing)
