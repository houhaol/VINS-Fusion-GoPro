#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import argparse
import rosbag
import rospy
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np

G_TO_MS2 = 9.80665
DEG_TO_RAD = np.pi / 180.0

def ns_to_ros_time(ns):
    return rospy.Time.from_sec(ns / 1e9)

def load_imu_csv(path, start_ns, end_ns):
    imu_msgs = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamp = int(row['timestamp [ns]'])
            if not (start_ns <= timestamp <= end_ns):
                continue

            imu_msg = Imu()
            imu_msg.header.stamp = ns_to_ros_time(timestamp)
            imu_msg.header.frame_id = "base_link"

            # Convert gyro from deg/s to rad/s
            imu_msg.angular_velocity.x = float(row['gyro x [deg/s]']) * DEG_TO_RAD
            imu_msg.angular_velocity.y = float(row['gyro y [deg/s]']) * DEG_TO_RAD
            imu_msg.angular_velocity.z = float(row['gyro z [deg/s]']) * DEG_TO_RAD

            # Convert accel from G to m/s²
            imu_msg.linear_acceleration.x = float(row['acceleration x [G]']) * G_TO_MS2
            imu_msg.linear_acceleration.y = float(row['acceleration y [G]']) * G_TO_MS2
            imu_msg.linear_acceleration.z = float(row['acceleration z [G]']) * G_TO_MS2

            # Orientation (quaternion)
            imu_msg.orientation.w = float(row['quaternion w'])
            imu_msg.orientation.x = float(row['quaternion x'])
            imu_msg.orientation.y = float(row['quaternion y'])
            imu_msg.orientation.z = float(row['quaternion z'])

            imu_msgs.append((imu_msg.header.stamp, imu_msg))
    return imu_msgs

def load_gps_csv_iphone(path, start_ns, end_ns):
    gps_msgs = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamp = int(row['time'])
            if not (start_ns <= timestamp <= end_ns):
                continue

            gps_msg = NavSatFix()
            gps_msg.header.stamp = ns_to_ros_time(timestamp)
            gps_msg.header.frame_id = "gps_link"

            gps_msg.latitude = float(row['latitude'])
            gps_msg.longitude = float(row['longitude'])
            gps_msg.altitude = float(row['altitude'])

            # Optional: set accuracy if available
            try:
                h_acc = float(row['horizontalAccuracy'])
                v_acc = float(row['verticalAccuracy'])
                gps_msg.position_covariance[0] = h_acc * h_acc
                gps_msg.position_covariance[4] = h_acc * h_acc
                gps_msg.position_covariance[8] = v_acc * v_acc
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            except:
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            gps_msgs.append((gps_msg.header.stamp, gps_msg))
    return gps_msgs

def load_gps_csv_columbus(path, start_ns, end_ns):
    import datetime
    gps_msgs = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:

            # Parse date and time
            # DATE: yymmdd, TIME: hhmmss
            date_str = row['DATE']
            time_str = row['TIME']
            try:
                dt = datetime.datetime.strptime(date_str + time_str, "%y%m%d%H%M%S")
                # Assume UTC; if not, adjust as needed
                timestamp = int(dt.timestamp() * 1e9)
            except Exception as e:
                continue  # skip if date/time is invalid

            if not (start_ns <= timestamp <= end_ns):
                continue

            # Parse latitude
            lat_str = row['LATITUDE N/S']
            if lat_str[-1] in ['N', 'S']:
                lat = float(lat_str[:-1])
                if lat_str[-1] == 'S':
                    lat = -lat
            else:
                continue  # skip if format is wrong

            # Parse longitude
            lon_str = row['LONGITUDE E/W']
            if lon_str[-1] in ['E', 'W']:
                lon = float(lon_str[:-1])
                if lon_str[-1] == 'W':
                    lon = -lon
            else:
                continue  # skip if format is wrong

            # Altitude
            try:
                alt = float(row['HEIGHT'])
            except:
                alt = 0.0

            gps_msg = NavSatFix()
            gps_msg.header.stamp = ns_to_ros_time(timestamp)
            gps_msg.header.frame_id = "gps_link"
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = alt
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            gps_msgs.append((gps_msg.header.stamp, gps_msg))
    return gps_msgs

def write_bag(output_path, imu_msgs, gps_msgs):
    with rosbag.Bag(output_path, 'w') as bag:
        for t, msg in sorted(imu_msgs + gps_msgs, key=lambda x: x[0].to_nsec()):
            if isinstance(msg, Imu):
                bag.write('/imu/data', msg, t)
            elif isinstance(msg, NavSatFix):
                bag.write('/fix', msg, t)
    print("[✓] Wrote %d IMU and %d GPS messages to %s" % (len(imu_msgs), len(gps_msgs), output_path))


def write_gps_bag(output_path, gps_msgs):
    with rosbag.Bag(output_path, 'w') as bag:
        for t, msg in sorted(gps_msgs, key=lambda x: x[0].to_nsec()):
            bag.write('/gps', msg, t)
    print("[✓] Wrote %d GPS messages to %s" % (len(gps_msgs), output_path))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert IMU and GPS CSV files to ROS bag")
    parser.add_argument('--imu_csv', help='Path to IMU CSV file')
    parser.add_argument('--gps_csv', help='Path to GPS CSV file')
    parser.add_argument('--start_time', type=float, required=True, help='Start time in seconds (e.g. 1751250633.72)')
    parser.add_argument('--end_time', type=float, required=True, help='End time in seconds (e.g. 1751250635.72)')
    parser.add_argument('--output_bag', default='imu_gps_filtered.bag', help='Output bag file name')
    args = parser.parse_args()

    # Convert seconds to nanoseconds for filtering
    start_time_ns = int(args.start_time * 1e9)
    end_time_ns = int(args.end_time * 1e9)

    # imu_data = load_imu_csv(args.imu_csv, start_time_ns, end_time_ns)
    # gps_data = load_gps_csv_iphone(args.gps_csv, start_time_ns, end_time_ns)
    gps_data = load_gps_csv_columbus(args.gps_csv, start_time_ns, end_time_ns)

    # if not imu_data:
    #     print("[!] No IMU messages in the given time range.")
    if not gps_data:
        print("[!] No GPS messages in the given time range.")

    # write_bag(args.output_bag, imu_data, gps_data)
    write_gps_bag(args.output_bag, gps_data)
