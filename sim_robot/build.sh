#!/bin/bash
# sim_robot 专用编译脚本（重要！）
#
# 这个 VM 镜像有个已知特性：ament_python 包编译后，控制台脚本(console_scripts)
# 只装到了 install/sim_robot/bin/，但 ros2 run / ros2 launch 实际是去
# install/sim_robot/lib/sim_robot/ 找可执行文件。所以单纯 colcon build 后，
# 你会看到 "No executable found"。
#
# 本脚本在 colcon build 之后补一步复制（bin -> lib/sim_robot），一步到位。
# 以后你改了 Python 代码，重新跑这个脚本即可：
#   bash ~/wheeltec_ros2/src/sim_robot/build.sh
set -e
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
echo "[1/2] colcon build --packages-select sim_robot"
colcon build --packages-select sim_robot
echo "[2/2] 复制可执行脚本 bin -> lib/sim_robot（VM 兼容补丁）"
mkdir -p install/sim_robot/lib/sim_robot
cp -a install/sim_robot/bin/. install/sim_robot/lib/sim_robot/
echo "完成。已就绪的可执行文件："
ls -1 install/sim_robot/lib/sim_robot/
