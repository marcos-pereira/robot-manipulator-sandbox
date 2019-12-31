# robot-manipulator-sandbox
This repository contains a simple project to control a robot manipulator on [CoppeliaSim](https://www.coppeliarobotics.com/) with ROS and [DQ Robotics](https://github.com/dqrobotics). The aim of the project is to explore some features of the DQ Robotics in python. 

The [pseudoinverse controller](https://github.com/marcos-pereira/robot-manipulator-sandbox/blob/master/scripts/controller.py) was borrowed from [this DQ Robotics example](https://github.com/dqrobotics/python-examples/blob/master/vrep_interface/vrep_interface_move_kuka.py). The controller itself was not changed. To follow some standards, we also used the design usually used to implement the kinematic controllers of DQ Robotics. We give all the credit to the [DQ Robotics Project](https://github.com/dqrobotics) team. 

This project was tested with Ubuntu 18.04, ROS Melodic Morenia and python3.

# What do I need to install?
1. Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu).

2. Install rospkg and catkin_pkg for python3
```
sudo pip3 install rospkg catkin_pkg
```

3. Follow the [DQ Robotics python documentation](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html) to install it for python.

4. Download [CoppeliaSim EDU](http://www.coppeliarobotics.com/ubuntuVersions).

# How to run the scripts?
1. Open a terminal session and execute `roscore`

2. Execute CoppeliaSim. 
```
cd <CoppeliaSimFolder>
./coppeliSim.sh
```

3. After CoppeliaSim has opened, open the scene [kuka_lw4.ttt](https://github.com/marcos-pereira/robot-manipulator-sandbox/blob/master/simulation_scenes/kuka_lw4.ttt) from within CoppeliaSim.

4. Press the Play Button on the CoppeliaSim scene.

5. Open a terminal and run the script [robot_controlling_node.py](https://github.com/marcos-pereira/robot-manipulator-sandbox/blob/master/scripts/robot_controlling_node.py) with 
```
python3 robot_controlling_node.py
```

6. The robot will move to the desired pose specified at [robot_controlling_node.py](https://github.com/marcos-pereira/robot-manipulator-sandbox/blob/master/scripts/robot_controlling_node.py).


