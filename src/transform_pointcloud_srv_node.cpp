// SPDX-FileCopyrightText: 2022 Hiroto Horimoto
// SPDX-License-Identifier: BSD-3-Clause

#include <transform_pointcloud_srv/transform_pointcloud_srv_node.hpp>

namespace transform_pointcloud_srv
{
  TransformPointCloudServerNode::TransformPointCloudServerNode() : Node("transform_pointcloud_srv_node")
  {
    RCLCPP_INFO(this->get_logger(), "Ready to transform_pointcloud.");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    service_ = this->create_service<transform_pointcloud_srv::srv::TransformPointcloud>("transform_pointcloud", 
      std::bind(&TransformPointCloudServerNode::transform, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  void TransformPointCloudServerNode::transform(const std::shared_ptr<rmw_request_id_t> request_header,
                                                const std::shared_ptr<transform_pointcloud_srv::srv::TransformPointcloud::Request> request,
                                                std::shared_ptr<transform_pointcloud_srv::srv::TransformPointcloud::Response> response)
  {
    (void)request_header;
    std::string topic_name = request->topic_name;
    std::string target_frame = request->target_frame;
    RCLCPP_INFO(this->get_logger(), "srv.request topic_name:[%s], target_frame:[%s]", topic_name.c_str(), target_frame.c_str());

    PointCloud2 msg;
    bool is_successful = rclcpp::wait_for_message(msg, this->shared_from_this(), topic_name, std::chrono::seconds(1));

    if (is_successful)
    {
      try
      {
        if (target_frame.empty() == false)
        {
          if (pcl_ros::transformPointCloud(target_frame, msg, response->cloud_out, *tf_buffer_) == false)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed pcl_ros::transformPointCloud target_frame:[%s]", target_frame.c_str());
            return;
          }
          response->cloud_out.header.frame_id = target_frame;
        }
      }
      catch(std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      }
      
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<transform_pointcloud_srv::TransformPointCloudServerNode>());
  rclcpp::shutdown();
  return 0;
}
