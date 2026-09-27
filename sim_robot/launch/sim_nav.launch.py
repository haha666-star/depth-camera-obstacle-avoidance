from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('sim_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf = os.path.join(pkg, 'urdf', 'robot.urdf')
    nav_params = os.path.join(pkg, 'config', 'sim_nav_params.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'sim_map.yaml')
    rviz = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    for f in (nav_params, map_yaml, urdf):
        if not os.path.exists(f):
            raise FileNotFoundError('缺少文件: ' + f)

    with open(urdf, 'r') as fh:
        robot_desc = fh.read()

    launch_rviz = LaunchConfiguration('launch_rviz')

    # 说明：本 launch 直接启动 Nav2 各节点，不再 include nav2_bringup 的 bringup_launch.py。
    # 原因1：该镜像 apt 版 nav2_bringup/bringup_launch.py 调用 launch 库的
    #   ReplaceString(condition=...)，而镜像内置 python3-launch 版本过低不支持 → 解析阶段崩溃。
    # 原因2：本镜像 nav2 为 ROS2 Humble，恢复服务已从旧的 nav2_recoveries 包
    #   合并进 nav2_behaviors（可执行文件 behavior_server）；直接用旧包名会报
    #   "package 'nav2_recoveries' not found"。手写启动可直接指定 Humble 正确包名。
    # 节点二进制仍用 apt 版（ABI 自洽，bt_navigator 不再 SIGSEGV）。
    # 各节点参数统一从 sim_nav_params.yaml 读取（顶层键即为节点名）。
    nav_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[nav_params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[nav_params]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav_params]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav_params]),
        # Humble 中恢复服务改名为 nav2_behaviors / behavior_server（旧版为 nav2_recoveries / recoveries_server）。
        # 节点名仍保持 recoveries_server，以匹配默认行为树里对恢复动作服务器的命名。
        Node(package='nav2_behaviors', executable='behavior_server', name='recoveries_server',
             output='screen', parameters=[nav_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             output='screen', parameters=[nav_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['map_server', 'amcl', 'planner_server',
                                         'controller_server', 'bt_navigator',
                                         'recoveries_server', 'waypoint_follower']}]),
    ]

    return LaunchDescription([
        # 可选关闭 RViz（无界面/调试时用 ros2 launch ... launch_rviz:=false）
        DeclareLaunchArgument('launch_rviz', default_value='true',
                             description='是否启动 RViz 可视化'),

        # 1) 仿真机器人：发 /scan /odom /tf，收 /cmd_vel
        Node(package='sim_robot', executable='sim_robot', name='sim_robot', output='screen'),

        # 2) 机器人模型描述（提供 base_link->laser 变换，RViz 显示小车）
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': robot_desc}]),

        # 3) Nav2 导航栈（手写启动，绕开 bringup_launch.py 的 launch-API 不兼容）
        *nav_nodes,

        # 4) RViz 可视化（含 "2D Pose Estimate" 和 "Nav2 Goal" 工具面板）
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz], output='screen',
             condition=IfCondition(launch_rviz)),
    ])
