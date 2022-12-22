# HOW TO USE ON ROS1 (melodic and noetic)

## Set Up
1. Download necessary package for this package.

```shell
$ sudo apt-get install ros-$ROS_DISTRO-pcl-ros # for pcl_ros
```

2. Download `transform_pointcloud_srv` package.

```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/HHorimoto/transform_pointcloud_srv.git
$ cd ~/catkin_ws
$ catkin_make
$ source ~/.bashrc
```

## Run Service
Run `transform_pointcloud_srv`

```shell
$ rosrun transform_pointcloud_srv transform_pointcloud_srv 
```

### request 
+ topic_name : topic (`PointCloud2`) name that you want to transfom. (type : `string`)
+ target_frame: link name of transform target. (type : `string`)

### response
+ cloud_out : topic name after transform. (type : `sensor_msgs/PointCloud2`)

## Run Client
Please write client node by yourself. You can find good tutorials at ROS wiki. 
+ [This](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29) is sample by C++.
+ [This](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) is sample by Python.

## Test
This package provides test bash for use with `Github Actions`.
This test confirms if this service returns `PointCloud2` with specified.
You can also do this test on your computer by following this command.

```shell
$ roscd transform_pointcloud_srv
$ bash -xv test/test.bash
$ echo $?
0 # It means success. if the number is "1", it means failure. 
```