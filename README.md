# transform_pointcloud_srv

**This package provides service on transform.**

![melodic workflow](https://github.com/HHorimoto/transform_pointcloud/actions/workflows/melodic.yml/badge.svg)
![noetic workflow](https://github.com/HHorimoto/transform_pointcloud/actions/workflows/noetic.yml/badge.svg)

## Requirement
+ ROS Melodic (on Ubuntu 18.04 LTS, build and run test on [Github Actions](.github/workflows/melodic.yml))
+ ROS Noetic (on Ubuntu 20.04 LTS, build and run test on [Github Actions](.github/workflows/noetic.yml))

## Set Up
Download `transform_pointcloud_srv` package.

```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/HHorimoto/transform_pointcloud_srv.git
$ cd ~/catkin_ws
$ catkin_make
```

## How To Use

1. Run `transform_pointcloud_srv`.

```shell
$ rosrun transform_pointcloud_srv transform_pointcloud_srv
```

2. Run example program (`client_py.py`).

I belive that you must change topic and link name in the [source code](https://github.com/HHorimoto/transform_pointcloud_srv/blob/main/src/transform_pointcloud_srv/client_py.py).

```shell
$ rosrun transform_pointcloud_srv client_py.py
```

### Brief Description of Example Source 

+ Plase see the service first.

```shell
$ roscd transform_pointcloud_srv/srv
$ cat TransformPointcloud.srv
string topic_name # 1st Request argument
string target_frame # 2nd Request argument
---
sensor_msgs/PointCloud2 cloud_out # Response value
```

1. Import `TransformPointcloud` service.

```py
from transform_pointcloud_srv.srv import TransformPointcloud
```

2. Wait until this service wakes up.

```py
rospy.wait_for_service('transform_pointcloud')
```

3. Call this service with Request and Get Response result.

```py
# call
client = rospy.ServiceProxy('transform_pointcloud', TransformPointcloud)
# get
resp1 = client("/camera/depth/color/points", "base_link")
```

4. Use Response result for your purposes like below.

```py
# print PointCloud data's frame_id, height, and width.
rospy.loginfo("info : frame_id = %s, height = %d, width = %d", resp1.cloud_out.header.frame_id, resp1.cloud_out.height, resp1.cloud_out.width)
```

## Test
This package provides test bash for use with `Github Actions`.
This test confirms if this service returns `PointCloud2` with specified link by rosbag.
You can also do this test on your computer by following this command.
I belive that you can see `Success` not `Fail`.

```shell
$ roscd transform_pointcloud_srv
$ bash -xv test/test_melodic.bash # or test_noetic.bash
# You can see result.
$ killall -9 rosmaster # kill roscore
```

## License

Distributed under the BSD-3-Clause License. See `LICENSE` for more information.