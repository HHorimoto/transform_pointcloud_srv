#!/usr/bin/env python

# SPDX-FileCopyrightText: 2022 Hiroto Horimoto
# SPDX-License-Identifier: BSD-3-Clause

import rospy
from transform_pointcloud_srv.srv import TransformPointcloud

def transform_pointcloud_srv(topic, frame):
    rospy.wait_for_service('transform_pointcloud')
    resp = None
    try:
        client = rospy.ServiceProxy('transform_pointcloud', TransformPointcloud)
        resp = client(topic, frame)
        if resp:
            rospy.loginfo("Run test succeeded")
    except rospy.ServiceException:
        rospy.loginfo("Run test failed")
        
def main():
    rospy.init_node('client')
    rospy.loginfo("Start TransformPointCloud")
    rospy.sleep(1)
    topic_name = rospy.get_param('~topic_name', 'points')
    target_frame = rospy.get_param('~target_frame', 'bar_link')
    transform_pointcloud_srv(topic_name, target_frame)

if __name__ == "__main__":
    main()