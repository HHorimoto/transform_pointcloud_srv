#!/usr/bin/env python2

# SPDX-FileCopyrightText: 2022 Hiroto Horimoto
# SPDX-License-Identifier: BSD-3-Clause

import roslaunch
import rospy
import rospkg
import sys
from std_msgs.msg import String
from transform_pointcloud_srv.srv import TransformPointcloud

class Client(object):
    def __init__(self, topic='/velodyne_points', frame='/base_link'):
        self.topic = topic
        self.frame = frame
    
    def test_node(self):
        rospy.wait_for_service('transform_pointcloud')
        try:
            client = rospy.ServiceProxy('transform_pointcloud', TransformPointcloud)
            resp1 = client(self.topic, self.frame)
            frame_id = resp1.cloud_out.header.frame_id
            if (frame_id != self.frame):
                return False
            return True
        except rospy.ServiceException:
            return False

def test_node():

    rospy.init_node('test_node', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    r = rospkg.RosPack()
    p = r.get_path('transform_pointcloud_srv')
    path = p + "/launch/test.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path])

    launch.start() # Launch test
    rospy.loginfo("Started")

    node = Client()
    result = node.test_node()

    launch.shutdown()
    if result:
        rospy.loginfo("Success")
        sys.exit(0)
    else:
        rospy.loginfo("Fail")
        sys.exit(1)
    

if __name__ == '__main__':
    test_node()