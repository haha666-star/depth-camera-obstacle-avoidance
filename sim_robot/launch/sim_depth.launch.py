from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('sim_robot')

    urdf = os.path.join(pkg, 'urdf', 'robot.urdf')
    rviz = os.path.join(pkg, 'rviz', 'sim_depth.rviz')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    launch_rviz = LaunchConfiguration('launch_rviz')
    use_rtabmap = LaunchConfiguration('use_rtabmap')
    use_slam = LaunchConfiguration('use_slam')
    fresh_db = LaunchConfiguration('fresh_db')
    rtabmap_db = LaunchConfiguration('rtabmap_db')
    rgb_texture = LaunchConfiguration('rgb_texture')
    rtabmap_proximity = LaunchConfiguration('rtabmap_proximity')
    rtabmap_opt_error = LaunchConfiguration('rtabmap_opt_error')
    grid_ray_tracing = LaunchConfiguration('grid_ray_tracing')
    laser_max = LaunchConfiguration('laser_max')
    camera_forward = LaunchConfiguration('camera_forward')
    camera_height = LaunchConfiguration('camera_height')
    camera_tf_offset = LaunchConfiguration('camera_tf_offset')

    # slam_toolbox 的官方 launch（找不到就跳过，不让 launch 直接崩）
    slam_launch = None
    try:
        slam_launch = os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch', 'online_async_launch.py')
        if not os.path.exists(slam_launch):
            slam_launch = None
    except Exception:
        slam_launch = None

    actions = [
        # ---- 开关 ----
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='是否启动 RViz（无显示器环境设 false，可用 ros2 run rviz2 单独开）'),
        DeclareLaunchArgument(
            'use_rtabmap', default_value='true',
            description='启动 RTAB-Map：用深度相机(RGB-D)实时重建 3D 点云地图'),
        DeclareLaunchArgument(
            'use_slam', default_value='false',
            description='启动 slam_toolbox：用 /scan 出干净的 2D 栅格地图(/map)。'
                        '注意 map->odom 会与 RTAB-Map 冲突，所以要它时请同时 use_rtabmap:=false'),
        DeclareLaunchArgument(
            'fresh_db', default_value='true',
            description='RTAB-Map 每次启动清空数据库。'
                        'true=每次从零建图（推荐，避免多次运行的地图叠在一起出现重影）；'
                        'false=接着上次的库继续建（保留历史地图）'),
        DeclareLaunchArgument(
            'rtabmap_db', default_value=os.path.expanduser('~/.ros/rtabmap_sim.db'),
            description='RTAB-Map 数据库文件路径（地图本体就存在这里）'),
        DeclareLaunchArgument(
            'rgb_texture', default_value='true',
            description='彩色相机画面是否用"世界坐标纹理"（给 RTAB-Map 提特征、做闭环用）。'
                        'false = 退回旧的距离伪彩（好看但提不出特征）'),
        # ---- RTAB-Map 位姿图优化开关（v7 新增）----
        DeclareLaunchArgument(
            'rtabmap_proximity', default_value='true',
            description='RTAB-Map 的"空间邻近链"(RGBD/ProximityBySpace)：把空间上靠得近的两个'
                        '关键帧连起来参与位姿图优化。真车里程计会漂，开着更稳；'
                        '但本仿真的 /odom 是【真值、零漂移】，开着只会把已经对齐的轨迹'
                        '"揉"变形 —— 实测点云会多鼓出 1.5m（房间外占比 0% → 4.2%）。'
                        '想拿到与真值严格对齐的点云就设 false'),
        DeclareLaunchArgument(
            'rtabmap_opt_error', default_value='3.0',
            description='RGBD/OptimizeMaxError：图优化后"误差比"超过它就整批驳回这次闭环。'
                        '实测本仿真里的闭环都是对的，但误差比 3.1~3.8 刚好压线被驳回；'
                        '调大（如 10.0）能让正确的闭环生效。调大前请先确认纹理唯一性好'),
        DeclareLaunchArgument(
            'grid_ray_tracing', default_value='true',
            description='Grid/RayTracing：给 /rtabmap/map 栅格做射线追踪，把"走过的空地方"'
                        '标成可通行。不开的话整张图只有障碍点和未知区（实测 free 仅 0.0%）'),

        # ---- 激光雷达射程（v8 新增，可调；默认仍是 5.0 不动）----
        DeclareLaunchArgument(
            'laser_max', default_value='5.0',
            description='/scan 的最大测距（米）。默认 5.0 与历史行为一致。'
                        '【调大它能显著减少未知区】用同一条 149s 真实轨迹离线重放实测：'
                        '5m→占用 2034 格/未知 67469；8m→2663/47404；12m→3020/38193。'
                        '【但不是无脑调大】房间是 16.2×12.2m 且高度对称，射程一长就会'
                        '一眼看到整间房的对称结构，扫描匹配容易"锁错方向"（VM 单轮实测 '
                        'laser_max=12.0 时 map->odom 漂到 −106°、图幅胀到 36×29m，'
                        '占用格连成墙的比例从 43% 掉到 14%）。'
                        '建议：先用默认 5.0；想要更大覆盖就试 8.0，并盯住 '
                        '`ros2 run tf2_ros tf2_echo map odom` 是否稳定。'
                        '调大后记得同步把 config/slam_params.yaml 的 max_laser_range '
                        '也放到同等量级（已预置 12.0 作为上限）。'
                        '本参数只影响 /scan（建图/显示用），不影响深度相机避障链路'),

        # ---- 相机装配偏移（v9 新增，把"第 3 个 bug"变成可做 A/B 的参数）----
        # 这两个值同时决定三件事：① 静态 TF base_link->camera 的平移；
        # ② 深度/彩色图像射线的起点；③ RViz 里相机小方块的画在哪。
        # 它们**不会**影响 sim_robot 自己发的 /points（那个已按世界坐标算好、直接挂 odom），
        # 但会通过 TF 影响 RTAB-Map 把深度图拼回地图时的位姿 —— 也就是影响
        # /rtabmap/cloud_map 与真值房间的对齐程度。
        DeclareLaunchArgument(
            'camera_forward', default_value='0.12',
            description='相机相对车头往前的偏移（米）。设 0.0 可做"没有这个偏移"的 A/B 对照'),
        DeclareLaunchArgument(
            'camera_height', default_value='0.30',
            description='相机离地高度（米）。设 0.0 可做"没有这个偏移"的 A/B 对照'),
        DeclareLaunchArgument(
            'camera_tf_offset', default_value='true',
            description='【v9 A/B 开关】静态 TF base_link->camera 里是否如实写装配偏移。'
                        'true=写（v7 修好后的行为）；false=平移写 0（复现第 3 个 bug）。'
                        '只影响 TF，不影响深度/彩色射线从哪出发 —— 做对照要用这个参数，'
                        '不要去设 camera_forward/camera_height=0（那会连传感器一起挪走，对照不成立）'),

        # 1) 仿真机器人：发 /scan /odom /camera/depth/image_raw /camera/rgb /camera_info /tf，收 /cmd_vel
        Node(package='sim_robot', executable='sim_robot', name='sim_robot', output='screen',
             parameters=[{'rgb_texture': ParameterValue(rgb_texture, value_type=bool),
                          'laser_max': ParameterValue(laser_max, value_type=float),
                          'camera_forward': ParameterValue(camera_forward, value_type=float),
                          'camera_height': ParameterValue(camera_height, value_type=float),
                          'camera_tf_offset': ParameterValue(camera_tf_offset, value_type=bool)}]),

        # 2) 机器人模型描述（提供 base_link->laser 变换，RViz 显示小车）
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', parameters=[{'robot_description': robot_desc}]),

        # 3) 深度相机避障节点：只看 /camera/depth/image_raw，发 /cmd_vel
        Node(package='sim_robot', executable='depth_avoider', name='depth_avoider', output='screen'),

        # 4) RTAB-Map：吃 /camera/rgb + /camera/depth + /camera/rgb/camera_info + /odom，
        #    重建真实 3D 点云地图 /rtabmap/cloud_map（真车 RealSense+避障链路就是这么建的）
        #    注意 ROS2 Humble 里 rtabmap 节点二进制在 rtabmap_slam 包（rtabmap_ros 只含消息/launch）。
        #    话题名必须用 remappings 改（节点内部订阅 /rgb/image、/depth/image、/rgb/camera_info、/odom）。
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             namespace='rtabmap',      # 必须带命名空间：本镜像的 rtabmap 节点在 /rtabmap 下（自检脚本按 /rtabmap 查找）
             output='screen',
             parameters=[{
                 'subscribe_depth': True,
                 'subscribe_rgb': True,
                 'subscribe_rgbd': False,
                 'subscribe_odom': True,
                 'subscribe_scan': False,
                 'frame_id': 'base_link',
                 'odom_frame_id': 'odom',
                 'map_frame_id': 'map',
                 'publish_tf': True,
                 'map_always_update': True,  # 每个新关键帧都更新地图并发布点云（generate_cloud_map 是无效参数，勿用）
                 'approx_sync': True,
                 'wait_for_transform': 0.2,
                 'topic_queue_size': 10,
                 'sync_queue_size': 30,
                 'qos_image': 1,          # 1=reliable，匹配仿真端 RELIABLE 发布
                 'qos_camera_info': 1,
                 'qos_odom': 1,
                 # 【v6】每次启动清库 + 独立库名。
                 # 默认库是 ~/.ros/rtabmap.db，会【跨多次运行一直累积】：
                 # 上一趟的地图和这一趟叠在一起，就成了"点云铺到 21m、12% 跑到房间外、
                 # 墙有重影"。清库后每趟都是干净的一间房。
                 'database_path': ParameterValue(rtabmap_db, value_type=str),
                 'delete_db_on_start': ParameterValue(fresh_db, value_type=bool),
                 # 【v7】位姿图优化开关（详见文件顶部对应 DeclareLaunchArgument 的说明）
                 'RGBD/ProximityBySpace': ParameterValue(rtabmap_proximity, value_type=str),
                 'RGBD/OptimizeMaxError': ParameterValue(rtabmap_opt_error, value_type=str),
                 # 【v7】2D 栅格：射线追踪出"可通行区域"，否则 /rtabmap/map 上只有障碍点
                 # ⚠⚠ rtabmap 的核心参数在 rtabmap_ros 里【全部声明为 string】，哪怕它表示的是
                 #   数字或布尔值（`ros2 param get` 会显示 `String value is: 3.0` / `... is: true`）。
                 #    类型传错的后果很坑：节点在【参数解析阶段】就抛 InvalidParameterTypeException
                 #    直接退出，终端只剩一行 what()，现象看起来像"RTAB-Map 根本没启动 / map 坐标系不存在"。
                 #    而且报错往往是"声明顺序里第一个不匹配的参数"，不一定是你最后改的那个 ——
                 #    2026-09-25 为此连踩三次（Grid/CellSize=0.05 → Grid/RayTracing=true →
                 #    RGBD/OptimizeMaxError=3.0），最后统一改成 str 才通过。
                 #    结论：给 rtabmap 传核心参数，一律 ParameterValue(x, value_type=str)。
                 'Grid/RayTracing': ParameterValue(grid_ray_tracing, value_type=str),
             }],
             remappings=[
                 ('rgb/image', '/camera/rgb/image_raw'),
                 ('depth/image', '/camera/depth/image_raw'),
                 ('rgb/camera_info', '/camera/rgb/camera_info'),
                 ('odom', '/odom'),
             ],
             condition=IfCondition(use_rtabmap)),

        # 5) 可选 RViz（有显示器时设 launch_rviz:=true）
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz], output='screen',
             condition=IfCondition(launch_rviz)),
    ]

    # 6) 可选 slam_toolbox：用现成的 /scan 出干净的 2D 栅格地图（有 /map 话题）
    if slam_launch is not None:
        actions.insert(4, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'rviz': 'false', 'use_sim_time': 'false',
                              'slam_params_file': slam_params}.items(),
            condition=IfCondition(use_slam)))

    return LaunchDescription(actions)
