from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('sim_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    urdf = os.path.join(pkg, 'urdf', 'robot.urdf')
    rviz = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    slam_launch = os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    # 校验文件存在，避免静默失败
    if not os.path.exists(slam_params):
        raise FileNotFoundError('缺少 slam_params.yaml: ' + slam_params)

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 1) 仿真机器人：发 /scan /odom /tf，收 /cmd_vel
        Node(package='sim_robot', executable='sim_robot', name='sim_robot', output='screen'),

        # 2) 机器人模型描述（让 RViz 显示小车 + 提供 base_link->laser 变换）
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': robot_desc}]),

        # 3) SLAM：在线异步建图（默认参数已匹配我们的话题 /scan /odom /base_link）
        #    use_sim_time:=false 很重要：我们用真实时钟，不是 Gazebo 仿真时钟
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'rviz': 'false', 'use_sim_time': 'false',
                              'slam_params_file': slam_params}.items()),

        # 4) RViz 可视化（地图 / 雷达 / TF / 导航面板）
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz], output='screen'),
    ])
