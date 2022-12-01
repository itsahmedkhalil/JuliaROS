# JuliaROS

Packages for communicating between Julia and ROS1 

## Getting Started (ROS1 noetic)

Create a workspace
```bash 
    mkdir -p catkin_ws/src
```

Clone this repo
```bash 
    cd catkin_ws/src && git clone https://github.com/itsahmedkhalil/JuliaROS
```

Build the packages and source the workspace
```bash 
    cd .. && catkin_ws && source devel/setup.bash
```

Run the gazebo launch file
```bash 
    roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```

Run the julia file
```bash 
    rosrun julia_pkg publisher.jl
```