![Screenshot](/capture.bmp)
Sample map built from [nsh_indoor_outdoor.bag](http://www.frc.ri.cmu.edu/~jizhang03/Datasets/nsh_indoor_outdoor.bag) (opened with [ccViewer](http://www.danielgm.net/cc/))

:white_check_mark: Tested with ROS Indigo and Velodyne VLP16. [(Screencast)](https://youtu.be/o1cLXY-Es54)

All sources were taken from [ROS documentation](http://docs.ros.org/indigo/api/loam_velodyne/html/files.html)

Ask questions [here](https://github.com/laboshinl/loam_velodyne/issues/3).

## How to build with catkin:

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/Chanuk-Yang/loam_velodyne.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```

## Running:

```
$ roslaunch loam_velodyne loam_velodyne_hy.launch
```

## In second terminal play sample velodyne data from [VLP16 rosbag](https://db.tt/t2r39mjZ):

```
$ rosbag play ~/Downloads/velodyne.bag 
```

## Or read from velodyne [VLP16 sample pcap](https://midas3.kitware.com/midas/folder/12979):

```
$ roslaunch velodyne_pointcloud VLP16_points.launch pcap:="/home/laboshinl/Downloads/velodyne.pcap"
```

## Save map file format PCD  

#### (activate this package when the rosbag is almost end)

```
$ rosrun pcl_ros pointcloud_to_pcd input:=/laser_cloud_surround
```

## Input & Output at overall SLAM algorithm

* ### Input (subscriber topic)

``` 
/vlp_center/velodyne_points : velodyne point cloud data (msg: sensor_msgs/PointCloud2)
/sensing/vehicle/rt : IMU sensor (msg: sensor_msgs/Imu)
```

* ### output (publisher topic)

```
/laser_cloud_surround : point cloud map (msg: sensor_msgs/PointCloud2)
```





---
[Quantifying Aerial LiDAR Accuracy of LOAM for Civil Engineering Applications.](https://ceen.et.byu.edu/sites/default/files/snrprojects/wolfe_derek.pdf) Derek Anthony Wolfe

[ROS & Loam_velodyne](https://ishiguro440.wordpress.com/2016/04/05/%E5%82%99%E5%BF%98%E9%8C%B2%E3%80%80ros-loam_velodyne/) 
