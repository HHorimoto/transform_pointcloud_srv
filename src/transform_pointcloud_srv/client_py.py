#!/usr/bin/env python

# SPDX-FileCopyrightText: 2022 Hiroto Horimoto
# SPDX-License-Identifier: BSD-3-Clause

import rospy
import std_msgs.msg
from transform_pointcloud_srv.srv import TransformPointcloud

def transform_pointcloud_srv(topic='/velodyne_points', frame='/base_link'):
    rospy.loginfo("TransformPointcloud Service")
    rospy.wait_for_service('transform_pointcloud')
    try:
        client = rospy.ServiceProxy('transform_pointcloud', TransformPointcloud)
        resp1 = client(topic, frame)
        # res : resp1.cloud_out
        rospy.loginfo("Get transformed PointCloud2")
        rospy.loginfo("info : frame_id = %s, height = %d, width = %d", 
                      resp1.cloud_out.header.frame_id, resp1.cloud_out.height, resp1.cloud_out.width)
    except rospy.ServiceException:
        rospy.loginfo("Fail.")
        
def main():
    rospy.init_node('client')
    rospy.loginfo("Start TransformPointCloud")
    rospy.sleep(1)
    transform_pointcloud_srv()

if __name__ == "__main__":
    main()