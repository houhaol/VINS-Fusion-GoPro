python generate_rosbag_from_imu_gps.py --gps_csv BF002/gps_traj/BF002_Columbus_GPS.csv --start_time 1751246942.04 --end_time 1751250633.72 --output_bag BF002/gps_columbus.bag


python slam_to_gps.py --traj /home/houhao/workspace/VINS-Fusion/BF002/slam_traj/slam_traj_01.txt --gps-csv /home/houhao/workspace/VINS-Fusion/BF002/gps_traj/BF002_iphone14_GPS.csv --output /home/houhao/workspace/VINS-Fusion/BF002/aligned_traj/aligned_traj_01.gpx --window 0,100,400,1000