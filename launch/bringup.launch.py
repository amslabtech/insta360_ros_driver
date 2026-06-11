from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Package share dir
    insta360_share = get_package_share_directory('insta360_ros_driver')

    bev_dir = get_package_share_directory('bev')
    create_costmap_dir = get_package_share_directory('create_costmap')

    bev_launch = os.path.join(bev_dir, 'launch', 'bev_node.launch.py')
    create_costmap_launch = os.path.join(create_costmap_dir, 'launch', 'create_costmap.launch.py')

    rviz_config_path = '/home/user/.rviz2/all.rviz'

    # ===== Launch Arguments =====
    equirectangular_arg = DeclareLaunchArgument(
        'equirectangular',
        default_value='true'
    )

    imu_filter_arg = DeclareLaunchArgument(
        'imu_filter',
        default_value='true'
    )

    equirectangular_config_arg = DeclareLaunchArgument(
        'equirectangular_config',
        default_value=os.path.join(
            insta360_share,
            'config',
            'equirectangular.yaml'
        )
    )

    imu_config_arg = DeclareLaunchArgument(
        'imu_config',
        default_value=os.path.join(
            insta360_share,
            'config',
            'imu_filter.yaml'
        )
    )

    # LaunchConfigurations
    equirectangular = LaunchConfiguration('equirectangular')
    imu_filter = LaunchConfiguration('imu_filter')
    equirectangular_config = LaunchConfiguration('equirectangular_config')
    imu_config = LaunchConfiguration('imu_config')

    # ===== Nodes =====

    # Publishes Compressed Images
    insta360_driver_node = Node(
        package='insta360_ros_driver',
        executable='insta360_ros_driver',
        name='insta360_ros_driver',
        output='log'
    )

    # Decodes Compressed Images
    decoder_node = Node(
        package='insta360_ros_driver',
        executable='decoder',
        name='image_decoder',
        output='log',
        parameters=[{
            'compressed_topic': '/dual_fisheye/image/compressed',
            'uncompressed_topic': '/dual_fisheye/image',
            'skip_frame': 0,
            'i_frame_only': False,
        }]
    )

    # Creates Equirectangular Images
    equirectangular_node = Node(
        condition=IfCondition(equirectangular),
        package='insta360_ros_driver',
        executable='equirectangular_cpp',
        name='equirectangular_node',
        output='screen',
        parameters=[equirectangular_config]
    )

    # Applies a Madgwick Filter to the 6-DOF IMU to get Orientation
    imu_filter_node = Node(
        condition=IfCondition(imu_filter),
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='log',
        parameters=[imu_config]
    )


    bev_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bev_launch)
    )

    create_costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(create_costmap_launch)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )


    static_tf_base_to_insta360 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_thetas',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_frame'],
     )

    # return LaunchDescription([
    #     equirectangular_arg,
    #     imu_filter_arg,
    #     equirectangular_config_arg,
    #     imu_config_arg,
    #     insta360_driver_node,
    #     decoder_node,
    #     equirectangular_node,
    #     imu_filter_node,
    #     bev_node,         
    #     create_costmap,    
    #     rviz,
    #     static_tf_base_to_insta360,
    # ])


    return LaunchDescription([
          equirectangular_arg,
          equirectangular_config_arg,
          insta360_driver_node,
          decoder_node,
          equirectangular_node,
          bev_node,         
          create_costmap,    
          rviz,
          static_tf_base_to_insta360,
    ])
