#!/usr/bin/env python3

import rospy
import serial
import re
import pynmea2
import math
import datetime
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import Twist, QuaternionStamped
from tf.transformations import quaternion_from_euler

def check_nmea_checksum(nmea_sentence):
    """Validates NMEA checksum to ensure data integrity."""
    split_sentence = nmea_sentence.split('*')
    if len(split_sentence) != 2:
        return False  # Missing checksum
    
    transmitted_checksum = split_sentence[1].strip()
    data_to_checksum = split_sentence[0][1:]  # Remove '$'
    
    checksum = 0
    for c in data_to_checksum:
        checksum ^= ord(c)
    
    return ("%02X" % checksum) == transmitted_checksum.upper()

def extract_nmea_sentence(line, sentence_type):
    """Extracts a valid NMEA sentence of the given type."""
    match = re.search(rf"(\${sentence_type},[^\r\n]*)", line)
    return match.group(1) if match else None

def parse_and_publish(nmea_sentence, publishers, use_gnss_time):
    """Parses NMEA sentence and publishes relevant ROS messages."""
    if not check_nmea_checksum(nmea_sentence):
        rospy.logwarn(f"Checksum failed for: {nmea_sentence}")
        return

    try:
        msg = pynmea2.parse(nmea_sentence)

        current_time = rospy.Time.now()  # Default to ROS time

        if isinstance(msg, pynmea2.GGA):  # GNSS Fix Data
            navsat_msg = NavSatFix()
            navsat_msg.header.stamp = current_time
            navsat_msg.header.frame_id = "navsat_link"
            navsat_msg.latitude = msg.latitude
            navsat_msg.longitude = msg.longitude

            # Ensure altitude is always a valid float
            try:
                navsat_msg.altitude = float(msg.altitude) if msg.altitude else float('nan')
            except ValueError:
                rospy.logwarn(f"Invalid altitude value received: {msg.altitude}")
                navsat_msg.altitude = float('nan')  # Default to NaN if conversion fails

            navsat_msg.status.status = NavSatStatus.STATUS_FIX if msg.gps_qual in [1, 2, 4, 5] else NavSatStatus.STATUS_NO_FIX
            navsat_msg.status.service = NavSatStatus.SERVICE_GPS

            publishers["gps/fix"].publish(navsat_msg)

            # GNSS Time Reference (Optional)
            if use_gnss_time and msg.timestamp:
                time_msg = TimeReference()
                time_msg.header.stamp = current_time
                time_msg.source = "GNSS"

                # Convert GNSS time from `datetime.time` to `datetime.datetime`
                now = datetime.datetime.utcnow().date()  # Get today's date
                full_datetime = datetime.datetime.combine(now, msg.timestamp)

                # Convert to ROS Time
                time_msg.time_ref = rospy.Time.from_sec(full_datetime.timestamp())
                publishers["gnss/time_reference"].publish(time_msg)

                # Quality Status Message
                gga_msg = String()
                gga_msg.data = f"Fix Quality: {msg.gps_qual}, Satellites: {msg.num_sats}, HDOP: {msg.horizontal_dil}, Altitude: {msg.altitude}m"
                publishers["gnss/gngga/quality"].publish(gga_msg)

        elif isinstance(msg, pynmea2.RMC):
            rmc_msg = String()
            rmc_msg.data = f"UTC: {msg.timestamp}, Speed: {msg.spd_over_grnd} knots, Course: {msg.true_course}°, Date: {msg.datestamp}"
            publishers["gnss/gnrmc/info"].publish(rmc_msg)

        elif isinstance(msg, pynmea2.VTG):
            twist_msg = Twist()
            twist_msg.linear.x = float(msg.spd_over_grnd_kmph) if msg.spd_over_grnd_kmph else 0.0
            twist_msg.angular.z = float(msg.true_track) if msg.true_track else 0.0
            publishers["gnss/gnvtg/speed_course"].publish(twist_msg)

        elif isinstance(msg, pynmea2.GSA):
            dop_msg = String()
            dop_msg.data = f"Fix Type: {msg.mode_fix_type}, PDOP: {msg.pdop}, HDOP: {msg.hdop}, VDOP: {msg.vdop}"
            publishers["gnss/gngsa/dop"].publish(dop_msg)

        elif isinstance(msg, pynmea2.GLL):
            gll_msg = String()
            gll_msg.data = f"Latitude: {msg.latitude} {msg.lat_dir}, Longitude: {msg.longitude} {msg.lon_dir}, Time: {msg.timestamp}"
            publishers["gnss/gngll/position"].publish(gll_msg)

        elif "HDT" in nmea_sentence:  # True Heading (Manually Parsing)
            match = re.match(r"\$.*HDT,([\d.]+),T\*.*", nmea_sentence)
            if match:
                heading = float(match.group(1))
                heading_msg = QuaternionStamped()
                heading_msg.header.stamp = current_time
                heading_msg.header.frame_id = "navsat_link"
                q = quaternion_from_euler(0, 0, math.radians(heading))
                heading_msg.quaternion.x = q[0]
                heading_msg.quaternion.y = q[1]
                heading_msg.quaternion.z = q[2]
                heading_msg.quaternion.w = q[3]
                publishers["gnss/hdt/heading"].publish(heading_msg)

    except pynmea2.nmea.ParseError as e:
        rospy.logwarn(f"Parse error for: {nmea_sentence} - {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

def gnss_parser_node():
    rospy.init_node('gnss_parser', anonymous=True)

    # ROS Publishers
    publishers = {
        "gps/fix": rospy.Publisher('/gps/fix', NavSatFix, queue_size=10),
        "gnss/gngga/quality": rospy.Publisher('/gnss/gngga/quality', String, queue_size=10),
        "gnss/gnrmc/info": rospy.Publisher('/gnss/gnrmc/info', String, queue_size=10),
        "gnss/gnvtg/speed_course": rospy.Publisher('/gnss/gnvtg/speed_course', Twist, queue_size=10),
        "gnss/gngsa/dop": rospy.Publisher('/gnss/gngsa/dop', String, queue_size=10),
        "gnss/gngll/position": rospy.Publisher('/gnss/gngll/position', String, queue_size=10),
        "gnss/hdt/heading": rospy.Publisher('/gnss/hdt/heading', QuaternionStamped, queue_size=10),
        "gnss/time_reference": rospy.Publisher('/gnss/time_reference', TimeReference, queue_size=10),
    }

    serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
    baud_rate = rospy.get_param('~baud_rate', 115200)
    use_gnss_time = rospy.get_param('~use_GNSS_time', True)

    rospy.loginfo(f"Opening serial port: {serial_port} at {baud_rate} baud")

    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            rospy.loginfo("GNSS parser node started. Reading data from serial port...")
            
            while not rospy.is_shutdown():
                line = ser.readline().decode('ascii', errors='replace').strip()
                
                for sentence in ["GNGGA", "GNRMC", "GNVTG", "GNGSA", "GNGLL", "HDT"]:
                    nmea_sentence = extract_nmea_sentence(line, sentence)
                    if nmea_sentence:
                        parse_and_publish(nmea_sentence, publishers, use_gnss_time)

    except serial.SerialException as e:
        rospy.logerr(f"Serial port error: {e}")

if __name__ == '__main__':
    try:
        gnss_parser_node()
    except rospy.ROSInterruptException:
        pass
