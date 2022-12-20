// SPDX-FileCopyrightText: 2022 Hiroto Horimoto
// SPDX-License-Identifier: BSD-3-Clause

#ifndef TRANSFORM_POINTCLOUD_SRV_HPP_
#define TRANSFORM_POINTCLOUD_SRV_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <transform_pointcloud_srv/srv/transform_pointcloud.hpp>

namespace transform_pointcloud_srv
{
    class TransformPointCloudServerNode : public rclcpp::Node
    {
        using PointCloud2 = sensor_msgs::msg::PointCloud2;

    public:
        TransformPointCloudServerNode();

        void transform(const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<transform_pointcloud_srv::srv::TransformPointcloud::Request> request,
                       std::shared_ptr<transform_pointcloud_srv::srv::TransformPointcloud::Response> response);

    private:

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::Service<transform_pointcloud_srv::srv::TransformPointcloud>::SharedPtr service_;
    };
}

#endif
