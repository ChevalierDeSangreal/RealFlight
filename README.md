
## 运行：
0. 设置ROS DOMAIN
export ROS_DOMAIN_ID=86

1. 启动IsaacSIm仿真+PX4

ISAACSIM_PYTHON /home/carlson/SimulatorSetup/submodules/PegasusSimulator/examples/1_px4_single_vehicle.py
ISAACSIM_PYTHON /home/carlson/SimulatorSetup/submodules/PegasusSimulator/examples/1_px4_single_raynor.py


source ~/Downloads/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash
source ~/Downloads/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash


ISAACSIM_PYTHON /home/wangzimo/Downloads/PegasusSimulator/examples/1_px4_single_raynor.py
ISAACSIM_PYTHON /home/wangzimo/Downloads/PegasusSimulator/examples/1_px4_single_vehicle.py

2. 启动DDS

MicroXRCEAgent udp4 -p 8888

3. (state machine)

cd /home/carlson/wangzimo/realflight_ws

source ./install/setup.bash

ros2 launch offboard_state_machine single_drone_test.launch.py


ros2 launch traj_test tflite_neural_control.launch.py
ros2 launch hover_test tflite_neural_control.launch.py

ros2 launch traj_test tflite_neural_control.launch.py > /home/carlson/wangzimo/tmplog.txt 2>&1
ros2 launch hover_test tflite_neural_control.launch.py > /home/carlson/wangzimo/tmplog.txt 2>&1

监视状态变换：

ros2 topic echo /state/state_drone_0 | awk '/data:/ {
    state = $2; 
    if (state != prev) {
        system("date +%H:%M:%S.%N | tr -d \\47\\n\\47"); 
        printf " State changed: %s -> %s", prev, state;
        if (state == 4) printf " (HOVER - position control!)";
        if (state == 5) printf " (TRAJ - body_rate control)";
        if (state == 6) printf " (END_TRAJ - position control!)";
        printf "\n";
        prev = state;
    }
}'

监视控制指令变化：
ros2 topic echo /fmu/in/offboard_control_mode | awk '
/position:/ {pos = $2}
/body_rate:/ {
    brate = $2;
    mode = pos brate;
    if (mode != prev) {
        system("date +%H:%M:%S.%N | cut -c1-12 | tr -d \"\\n\"");
        if (brate == "true")
            printf " -> BODY_RATE control (TRAJ)\n";
        else if (pos == "true")
            printf " -> POSITION control (Normal)\n";
        prev = mode;
    }
}'

监视角速度指令：

ros2 topic echo /fmu/in/vehicle_rates_setpoint

监视控制环：

ros2 topic echo /fmu/in/offboard_control_mode

重新编译：

cd ~/wangzimo/realflight_ws
colcon build --packages-select traj_test
source install/setup.bash

cd ~/wangzimo/realflight_ws
colcon build --packages-select offboard_state_machine
source install/setup.bash


cd ~/wangzimo/realflight_ws
colcon build --packages-select hover_test
source install/setup.bash




ros2 launch traj_test traj_test.launch.py drone_id:=0


## 环境配置：

Pegasus对应地址：
https://github.com/Temasek-Dynamics/PegasusSimulator.git