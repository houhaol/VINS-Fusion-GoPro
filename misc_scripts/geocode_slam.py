import argparse
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import rosbag
import sensor_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import pandas as pd
import datetime

# WGS84 ellipsoid constants
a = 6378137.0
f = 1 / 298.257223563
e2 = f * (2 - f)

def ecef_to_gps(x, y, z):
    """Convert ECEF (x, y, z) to WGS84 (lat, lon, alt)."""
    # WGS84 constants
    b = a * (1 - f)
    ep = np.sqrt((a**2 - b**2) / b**2)
    p = np.sqrt(x**2 + y**2)
    th = np.arctan2(a * z, b * p)
    lon = np.arctan2(y, x)
    lat = np.arctan2(z + ep**2 * b * np.sin(th)**3, p - e2 * a * np.cos(th)**3)
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    alt = p / np.cos(lat) - N
    lat = np.rad2deg(lat)
    lon = np.rad2deg(lon)
    return lat, lon, alt

def gps_to_ecef(lat, lon, alt):
    """Convert WGS84 (lat, lon, alt) to ECEF (x, y, z) in meters."""
    lat = np.deg2rad(lat)
    lon = np.deg2rad(lon)
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (1 - e2) + alt) * np.sin(lat)
    return np.array([x, y, z])


def read_gps_from_bag(bag_path, topic='/gps/fix'):
    """Read GPS (timestamp, lat, lon, alt) from rosbag."""
    gps_data = []
    with rosbag.Bag(bag_path) as bag:
        for topic_, msg, t in bag.read_messages(topics=[topic]):
            if msg.status.status < 0:
                continue  # skip invalid fix
            ts = msg.header.stamp.to_sec()
            gps_data.append((ts, msg.latitude, msg.longitude, msg.altitude))
    return gps_data

def read_gps_from_csv_iphone(csv_path):
    """Read GPS (timestamp, lat, lon, alt) from CSV file with header."""
    gps_data = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Use 'time' as timestamp (float), 'latitude', 'longitude', 'altitude'
            try:
                ts = float(row['time'])
                lat = float(row['latitude'])
                lon = float(row['longitude'])
                alt = float(row['altitude'])
                gps_data.append((ts, lat, lon, alt))
            except Exception:
                continue
    return gps_data

def read_gps_from_csv_columbs(csv_path):
    df = pd.read_csv(csv_path)
    points = []
    for _, row in df.iterrows():
        try:
            lat_str = str(row['LATITUDE N/S']).strip()
            lon_str = str(row['LONGITUDE E/W']).strip()
            # Remove trailing direction letter if present
            if lat_str[-1] in ['N', 'S']:
                lat_val = lat_str[:-1]
            else:
                lat_val = lat_str
            if lon_str[-1] in ['E', 'W']:
                lon_val = lon_str[:-1]
            else:
                lon_val = lon_str
            lat = float(lat_val)
            lon = float(lon_val)
            ele = float(row.get('HEIGHT', 0))
            date_str = row.get('DATE', '')
            time_str = row.get('TIME', '')
            if date_str and time_str:
                try:
                    # dt = datetime.strptime(str(date_str) + ' ' + str(time_str), '%Y-%m-%d %H:%M:%S')
                    dt = datetime.datetime.strptime(str(date_str) + str(time_str), "%y%m%d%H%M%S")
                    timestamp = int(dt.timestamp() * 1e9)
                except Exception:
                    timestamp = None
            else:
                timestamp = None
            points.append((timestamp, lat, lon, ele))
        except Exception:
            continue
    return points


def read_slam_traj_mins(traj_path):
    """Read SLAM trajectory (timestamp, tx, ty, tz, ...) from txt file (MINS format)."""
    traj = []
    with open(traj_path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split()
            ts = float(parts[0])
            tx, ty, tz = map(float, parts[1:4])
            traj.append((ts, np.array([tx, ty, tz])))
    return traj

def read_slam_traj_vins(traj_path):
    """Read SLAM trajectory (timestamp, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w) from VINS format."""
    traj = []
    with open(traj_path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            pos_x, pos_y, pos_z = map(float, parts[1:4])
            # ori_x, ori_y, ori_z, ori_w = map(float, parts[4:8])  # Not used here
            traj.append((ts, np.array([pos_x, pos_y, pos_z])))
    return traj

def match_by_timestamp(slam_traj, gps_data, max_dt=0.1):
    """Match SLAM and GPS points by closest timestamp within max_dt seconds."""
    gps_times = np.array([g[0]/1e9 for g in gps_data])
    matches = []
    # import pdb; pdb.set_trace()
    for ts, pos in slam_traj:
        # convert ts to nanoseconds
        idx = np.argmin(np.abs(gps_times - ts))
        dt = abs(gps_times[idx] - ts)
        if dt < max_dt:
            matches.append((pos, gps_data[idx][1:]))  # (slam_xyz, (lat, lon, alt))
    return matches

def compute_umeyama(src, dst):
    """Compute similarity (rotation, translation, scale) using Umeyama algorithm."""
    src = np.asarray(src).T  # 3xN
    dst = np.asarray(dst).T
    mu_src = np.mean(src, axis=1, keepdims=True)
    mu_dst = np.mean(dst, axis=1, keepdims=True)
    src_centered = src - mu_src
    dst_centered = dst - mu_dst
    cov = dst_centered @ src_centered.T / src.shape[1]
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R_ = U @ S @ Vt
    var_src = np.sum(src_centered ** 2) / src.shape[1]
    scale = np.trace(np.diag(D) @ S) / var_src
    t = mu_dst - scale * R_ @ mu_src
    return scale, R_, t

def main():
    parser = argparse.ArgumentParser(description='Geo-anchor SLAM trajectory using GPS from rosbag or CSV.')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--bag', help='Path to rosbag file with /gps/fix')
    group.add_argument('--gps-csv', help='Path to GPS CSV file (with time,latitude,longitude,altitude columns)')
    parser.add_argument('--traj', required=True, help='Path to SLAM trajectory txt file')
    parser.add_argument('--output', required=True, help='Output file for geo-anchored trajectory')
    parser.add_argument('--max-dt', type=float, default=0.1, help='Max timestamp difference for matching (s)')
    parser.add_argument('--window', type=str, action='append', default=None, help='Time window for matching, format: start,end (relative to first SLAM timestamp, in seconds). Can be specified multiple times.')
    args = parser.parse_args()
    # Read GPS data from the selected source
    if args.bag:
        gps_data = read_gps_from_bag(args.bag)
    elif args.gps_csv:
        # gps_data = read_gps_from_csv_iphone(args.gps_csv)
        gps_data = read_gps_from_csv_columbs(args.gps_csv)
    else:
        print('Either --bag or --gps-csv must be specified.')
        return
    # Change this to read_slam_traj_vins(args.traj) if using VINS format
    slam_traj = read_slam_traj_mins(args.traj)
    if not slam_traj:
        print('No SLAM trajectory data found.')
        return
    first_ts = slam_traj[0][0]
    # Parse windows
    windows = []
    if args.window:
        tmp_window = [float(i) for i in args.window[0].split(',')]
        # every two elements form a window
        for i in range(0, len(tmp_window), 2):
            try:
                s, e = tmp_window[i], tmp_window[i + 1]
                abs_s = first_ts + s
                abs_e = first_ts + e
                windows.append((abs_s, abs_e))
            except Exception:
                print(f"Invalid window format. Should be start,end")
                return
    else:
        # fallback to old behavior: one window from start/end
        abs_start = first_ts + getattr(args, 'start', 0.0)
        abs_end = first_ts + getattr(args, 'end', 1e10)
        windows.append((abs_start, abs_end))
    # Filter SLAM traj for all windows
    slam_traj_window = []
    for abs_start, abs_end in windows:
        slam_traj_window.extend([(ts, pos) for ts, pos in slam_traj if abs_start <= ts <= abs_end])
    matches = match_by_timestamp(slam_traj_window, gps_data, max_dt=args.max_dt)
    if len(matches) < 3:
        print('Not enough matches between SLAM and GPS for alignment in the selected timeframe.')
        return
    else:
        print("Number of matches is {}".format(len(matches)))
    slam_xyz = [m[0] for m in matches]
    gps_ecef = [gps_to_ecef(*m[1]) for m in matches]
    scale, R_, t = compute_umeyama(slam_xyz, gps_ecef)


    # Transform all SLAM points and output as WGS84 (apply all time windows)
    with open(args.traj) as f:
        lines = f.readlines()
    with open(args.output, 'w') as fout:
        fout.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        fout.write('<gpx version="1.1" creator="slam_to_gps.py" xmlns="http://www.topografix.com/GPX/1/1">\n')
        fout.write('  <trk>\n    <name>SLAM to GPS Trajectory</name>\n    <trkseg>\n')
        for line in lines:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split()
            ts = float(parts[0])
            # Only include points in any window
            # in_window = False
            # for abs_start, abs_end in windows:
            #     if abs_start <= ts <= abs_end:
            #         in_window = True
            #         break
            # if not in_window:
            #     continue
            tx, ty, tz = map(float, parts[1:4])
            xyz = np.array([tx, ty, tz])
            xyz_ecef = scale * R_ @ xyz + t.flatten()
            lat, lon, alt = ecef_to_gps(*xyz_ecef)
            # Format timestamp as ISO8601 (UTC)
            import datetime
            time_str = datetime.datetime.utcfromtimestamp(ts).isoformat() + 'Z'
            fout.write(f'      <trkpt lat="{lat:.8f}" lon="{lon:.8f}">\n')
            fout.write(f'        <ele>{alt:.3f}</ele>\n')
            fout.write(f'        <time>{time_str}</time>\n')
            fout.write('      </trkpt>\n')
        fout.write('    </trkseg>\n  </trk>\n</gpx>\n')

    print(f"Geo-anchored trajectory (GPX) written to {args.output}")

if __name__ == '__main__':
    main()
