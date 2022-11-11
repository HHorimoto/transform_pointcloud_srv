// SPDX-FileCopyrightText: 2022 Hiroto Horimoto
// SPDX-License-Identifier: BSD-3-Clause

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <transform_pointcloud_srv/TransformPointcloud.h>
#include <sensor_msgs/PointCloud2.h>

class TfPointServer {
    public:
        TfPointServer();
        ~TfPointServer() {};
        bool transform(transform_pointcloud_srv::TransformPointcloud::Request &req, 
                        transform_pointcloud_srv::TransformPointcloud::Response &res);
        void loop();

    private:
        ros::NodeHandle nh;
        ros::ServiceServer service;
        tf::TransformListener tf_listener;
};

TfPointServer::TfPointServer()
{
    ROS_INFO("Ready to transform pointcloud");

    service = nh.advertiseService("transform_pointcloud", &TfPointServer::transform, this);
}

bool TfPointServer::transform(transform_pointcloud_srv::TransformPointcloud::Request &req, 
                        transform_pointcloud_srv::TransformPointcloud::Response &res)
{
    // http://docs.ros.org/en/melodic/api/roscpp/html/namespaceros_1_1topic.html
    boost::shared_ptr<sensor_msgs::PointCloud2 const> pc_data = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.topic_name.data, nh);
    
    if (pc_data) {
        try
        {
            if (req.target_frame.data.empty() == false)
            {
                if (tf_listener.waitForTransform(req.target_frame.data, pc_data->header.frame_id, ros::Time(0), ros::Duration(4.0)) == false) {
                    ROS_ERROR("Can't wait for transform");
                    return false;
                }
                if (pcl_ros::transformPointCloud(req.target_frame.data, *pc_data, res.cloud_out, tf_listener) == false)
                {
                    ROS_ERROR("Failed pcl_ros::transformPointCloud target_frame:[%s]", req.target_frame.data.c_str());
                    return false;
                }
                ROS_INFO("Processed successfully");
                return true;
            }
            return false;
        }
        catch (std::exception &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
    }
    ROS_ERROR("Can't get message");
    return false;
}

void TfPointServer::loop()
{
    ros::Rate loop_rate(30);                                                         
    while(ros::ok()) {                                                           
        ros::spinOnce();                                          
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{                                                    
    ros::init(argc, argv, "transform_pointcloud_srv");

    TfPointServer tfsvr;
    tfsvr.loop();
    return 0;
}