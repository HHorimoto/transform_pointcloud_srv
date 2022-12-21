import launch
import launch_ros.actions

def generate_launch_description():
    package_name = 'transform_pointcloud_srv'
    topic_in = 'points'
    from_frame = 'foo_link'
    to_frame = 'bar_link'
    point_x = 1.0
    point_y = 1.0
    point_z = 1.0
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=package_name,
            executable='transform_pointcloud_srv_node',
            name='transform_pointcloud_srv_node',
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments = ["0", "0", "1.0", "0", "0", "0", to_frame, from_frame]
        ),
        launch_ros.actions.Node(
            package=package_name,
            executable='test_talker.py',
            name='test_talker',
            parameters=[{'topic_name':topic_in},
                        {'frame_id':to_frame},
                        {'x':point_x},
                        {'y':point_y},
                        {'z':point_z}]
        ),
        launch_ros.actions.Node(
            package=package_name,
            executable='test_client.py',
            name='test_client',
            parameters=[{'topic_name':topic_in}]
        )
    ])