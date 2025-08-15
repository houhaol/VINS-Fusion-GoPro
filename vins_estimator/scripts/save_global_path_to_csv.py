#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import csv
import signal
import sys
import argparse


latest_path = None
output_filename = None

def save_path_to_csv(path_msg, filename):
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp_ns', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
        last = None
        for pose in path_msg.poses:
            if last:
                dt = abs(pose.header.stamp.to_sec() - last.header.stamp.to_sec())
                dx = pose.pose.position.x - last.pose.position.x
                dy = pose.pose.position.y - last.pose.position.y
                dz = pose.pose.position.z - last.pose.position.z
                dist = (dx**2 + dy**2 + dz**2)**0.5
                if dt < 1e-6 and dist < 1e-4:
                    continue
            writer.writerow([
                int(pose.header.stamp.to_sec() * 1e9),
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z
            ])
            last = pose

def path_callback(msg):
    global latest_path
    latest_path = msg

def shutdown_hook():
    if latest_path:
        save_path_to_csv(latest_path, output_filename)
        rospy.loginfo(f"Trajectory saved to {output_filename}")
    else:
        rospy.logwarn("No path message received, nothing saved.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Save global path to CSV.')
    parser.add_argument('--output', '-o', type=str, default='trajectory.csv', help='Output CSV filename (default: trajectory.csv)')
    args = parser.parse_args()
    global output_filename
    output_filename = args.output

    rospy.init_node('save_global_path_to_csv')
    rospy.Subscriber('/globalEstimator/global_path', Path, path_callback)
    rospy.on_shutdown(shutdown_hook)
    rospy.loginfo(f"Waiting for /globalEstimator/global_path messages. Press Ctrl+C to save and exit. Output file: {output_filename}")
    rospy.spin()