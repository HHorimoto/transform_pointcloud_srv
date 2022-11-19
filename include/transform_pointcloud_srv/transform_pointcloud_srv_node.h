// SPDX-FileCopyrightText: 2022 Hiroto Horimoto
// SPDX-License-Identifier: BSD-3-Clause

#ifndef TRANSFORM_POINTCLOUD_SRV_H__
#define TRANSFORM_POINTCLOUD_SRV_H__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <transform_pointcloud_srv/TransformPointcloud.h>

namespace transform_pointcloud_srv
{
    class TfPointServerNode
    {
    public:
        TfPointServerNode();
        ~TfPointServerNode();
        bool transform(transform_pointcloud_srv::TransformPointcloud::Request &req,
                       transform_pointcloud_srv::TransformPointcloud::Response &res);
        void loop();

    private:
        ros::NodeHandle nh;
        ros::ServiceServer service;
        tf::TransformListener tf_listener;
    };
}

#endif // TRANSFORM_POINTCLOUD_SRV_H__