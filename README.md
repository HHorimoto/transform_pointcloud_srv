# transform_pointcloud_srv

**This package provides service on transform.**

![melodic workflow](https://github.com/HHorimoto/transform_pointcloud/actions/workflows/melodic.yml/badge.svg)
![noetic workflow](https://github.com/HHorimoto/transform_pointcloud/actions/workflows/noetic.yml/badge.svg)

## Requirement
+ ROS Melodic (on Ubuntu 18.04 LTS, build and run test on Github Actions)
+ ROS Noetic (on Ubuntu 20.04 LTS, only build test on Github Actions)

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
std_msgs/String topic_name # 1st Request argument
std_msgs/String target_frame # 2nd Request argument
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
frame_msg = std_msgs.msg.String()
frame_msg.data = "base_link"
topic_name = std_msgs.msg.String()
topic_name.data = "/camera/depth/color/points"
# get
resp1 = client(topic_name, frame_msg)
```

4. Use Response result for your purposes like below.

```py
# print PointCloud data's frame_id, height, and width.
rospy.loginfo("info : frame_id = %s, height = %d, width = %d", resp1.cloud_out.header.frame_id, resp1.cloud_out.height, resp1.cloud_out.width)
```

## License

Distributed under the BSD-3-Clause License. See `LICENSE` for more information.