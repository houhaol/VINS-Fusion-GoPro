# GoPro-SLAM using GoPro13

## 1. Calibration
Use https://github.com/urbste/OpenImuCameraCalibrator/blob/master/docs/gopro_calibration.md to calibrate camera. \
Notes: \
1.1 Disable hypersmooth mode in GoPro Video settings. \
1.2 Record three videos for calibration. The first is to calibrate the camera. Move SLOWLY around the board. We do not want motion blur or the rolling shutter to influence the result. Record for about 20-30s. LESS THAN 30s. \
1.3 Run the calibration which is installed via docker. \
1.4 Run `scripts/convert.py` to get the transformation matrix from camera to imu. 

## 2. VINS in Docker

Docker image pull \
`docker pull jianchong/vins-fusion`

Check this website https://wiki.ros.org/docker/Tutorials/GUI for using noVNC. RVIZ will be directed to a web`http://localhost:8080/vnc.html` 

```
docker run -d --rm --net=ros \
   --env="DISPLAY_WIDTH=3000" --env="DISPLAY_HEIGHT=1800" --env="RUN_XTERM=no" \
   --name=novnc -p=8080:8080 \
   theasp/novnc:latest
docker run -d --net=ros --name roscore osrf/ros:noetic-desktop-full roscore
```

Docker run container 
`docker run -it --net=ros --env="DISPLAY=novnc:0.0" --env="ROS_MASTER_URI=http://roscore:11311" jianchong/vins-fusion /bin/bash`