# 添加使用手柄遥控
sudo apt-get install ros-noetic-joy
sudo apt-get install ros-noetic-teleop-twist-joy
sudo apt-get install ros-noetic-joy-teleop

#################################################################
# legged control 工作流程
1. 启动robot
export ROBOT_TYPE=a1

simulation
roslaunch legged_unitree_description empty_world.launch
or realworld
roslaunch legged_unitree_hw legged_unitree_hw.launch

2.启动控制器
export ROBOT_TYPE=a1
simulation
roslaunch legged_controllers load_controller.launch cheater:=true
or realworld
roslaunch legged_controllers load_controller.launch cheater:=false

ps:输入步态

3.启动遥控
roslaunch legged_controllers joy_teleop.launch

4.开启运动
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" 
