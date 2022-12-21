#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2022 Hiroto Horimoto
# SPDX-License-Identifier: BSD-3-Clause

import sys

from transform_pointcloud_srv.srv import TransformPointcloud
import rclpy
from rclpy.node import Node


class TestClientAsync(Node):

    def __init__(self):
        super().__init__('test_client_async')
        self.cli = self.create_client(TransformPointcloud, 'transform_pointcloud')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TransformPointcloud.Request()
        self.topic_name = self.declare_parameter('topic_name', 'points').get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'foo_link').get_parameter_value().string_value

    def send_request(self):
        self.req.topic_name = self.topic_name
        self.req.target_frame = self.frame_id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    test_client = TestClientAsync()
    response = test_client.send_request()
    if (response.cloud_out):
        test_client.get_logger().info('Run test succeeded')
    else:
        test_client.get_logger().info('Run test failed')

    test_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()