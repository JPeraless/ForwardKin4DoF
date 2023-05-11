# Simulation for SSRT's rover robot arm in Gazebo and RViz

- Install Ubuntu 20.04
- Install ROS Noetic (full version)
- Install MoveIt (by completing the tutorial at https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
- sudo apt-get install ros-noetic-joint-trajectory-controller
- source ~/ws_moveit/devel/setup.bash
- cd ws_moveit/src
- git clone https://github.com/JPeraless/rover-simulation.git
- Drag sw_exported_urdf and sw_exported_arm_sim up one directory into ws_moveit/src
- catkin build
- source ~/ws_moveit/devel/setup.bash
- roslaunch sw_exported_arm_sim full_arm_sim.launch
