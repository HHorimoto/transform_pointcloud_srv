# HOW TO USE ON ROS2 (foxy)

## Set Up
1. Download necessary package for this package.

```shell
$ cd ~/ros2_ws/src/
$ git clone https://github.com/ros-perception/perception_pcl.git -b foxy-devel
$ cd ~/ros2_ws
$ colcon build
$ source ~/.bashrc
```

2. Download `transform_pointcloud_srv` package.

```shell
$ cd ~/ros2_ws/src/
$ git clone https://github.com/HHorimoto/transform_pointcloud_srv.git -b foxy-devel
$ cd ~/ros2_ws
$ colcon build --allow-overriding pcl_conversions
$ source ~/.bashrc
```

## Run
Run `transform_pointcloud_srv`

```shell
$ ros2 run transform_pointcloud_srv transform_pointcloud_srv_node
```

### request 
+ topic_name : topic (`PointCloud2`) name that you want to transfom. (type : `string`)
+ target_frame: link name of transform target. (type : `string`)

### response
+ cloud_out : topic name after transform. (type : `sensor_msgs/PointCloud2`)

## Run Client
Please write client node by yourself. You can find good tutorials at ROS2 wiki. 
+ [This](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html) is sample by C++.
+ [This](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) is sample by Python.

## Test
This package provides test bash for use with `Github Actions`.
This test confirms if this service returns `PointCloud2` with specified.
You can also do this test on your computer by following this command.

```shell
$ cd ros2_ws/src/transform_pointcloud_srv/
$ bash -xv test/test.bash
$ echo $?
0 # It means success. if the number is "1", it means failure. 
``` 