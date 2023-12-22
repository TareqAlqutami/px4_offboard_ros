# Offboard PX4 control using MAVROS

## Perquisite

   * Install [ROS](https://wiki.ros.org/noetic/Installation/Ubuntu)
   * Download and build [PX4-Autopilot](https://docs.px4.io/main/en/dev_setup/building_px4.html)
   * Install [Mavros](https://docs.px4.io/main/en/ros/mavros_installation.html)
   * Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu) (optional)

Tested with
   * Ubuntu: 20.04
   * ROS: Noetic with Gazebo 11 (classic)
   * Python 3.8
   * PX4 main branch on Dec 2024 (version 1.15 beta)
  
## Setup
- create a ros catkin workspace and build it with either catkin_make or catkin build.
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make
    ```
- download this repository and build the workspace again
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/TareqAlqutami/px4_offboard_ros
    catkin_make
    ```

## Run
Assuming that PX4 autopilot is located in `~/PX4-Autopilot` and the ros catkin workspace is in `~/catkin_ws`

 - Source the Gazebo sitl environment
    ```bash
    cd ~/PX4-Autopilot
    source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

    # expose px4 commands to shell (optional)
    export PATH=$PATH:$(pwd)/build/px4_sitl_default/bi
    ```

 - source ros workspace in the same terminal
    ```bash
    cd ~/catkin_ws
    source devel/setup.bash
    ``` 

 - Run Any of the example codes as explained below


### Code
- `offboard_test_node.py` Python script: 
    
    it copied from px4 mavros documentation and shows simple code on how to use mavros with px4. The script switches to offboard mode, arms the drone then takes off and moves to position (x,y,z) = (0.5,0.0,1.5) using position control.
    You will need to run the gazebo mavros sitl first before running the script
    ``` bash
    # in first terminal, source px4 gazebo then
    roslaunch px4 mavros_posix_sitl.launch

    # in secoond terminal, source ros workspace then
    rosrun px4_offboard_ros offboard_test_node.py
    ```
- `offb_node.cpp` C++ code: 

    it was also copied from px4 mavros documentation and does the same functionality. to run the code
    ``` bash
    # in first terminal, source px4 gazebo then
    roslaunch px4 mavros_posix_sitl.launch

    # in secoond terminal, source ros workspace then
    rosrun px4_offboard_ros offb_node
    ```

- `mavros_offboard_control.py` Python script: 
    shows how to setup use MAVROS in ros code. It subscribes to essential topics and shows how to switch to offboard mode, takeoff and send different type of setpoints. Change the  `control_mode` variable in the code to change the type of controller and setpoints to test. This code is not meant to be run in actual drone since it asks for unsafe setpoints. The code sends the setpoints to px4 in a separate thread. A huge portion of the code was adopted from the [mavros integration test code](https://github.com/PX4/PX4-Autopilot/tree/main/integrationtests/python_src/px4_it/mavros) in px4.

    Gazebo sitl with mavros and this script are combined in `start_offboard_sitl` launch file.
    ``` bash
    # in the same terminal, source px4 gazebo and ros workspace then
    roslaunch px4_offboard_ros start_offboard_sitl.launch
    ```