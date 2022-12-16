# Project Collie (Autonomous Kitting Robot)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build Status](https://github.com/lilnpuma/ProjectCollie/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/lilnpuma/ProjectCollie/actions/workflows/build_and_coveralls.yml)
[![Coverage Status](https://coveralls.io/repos/github/lilnpuma/ProjectCollie/badge.svg?branch=main)](https://coveralls.io/github/lilnpuma/ProjectCollie?branch=master)
## Overview
---
The aim of the project is to implement a solution for automation of warehouses using manipulator robot to perform kitting operations autonomously. Robotic Kitting is the creation of product assortments, most often for retail sale. High speed picking robots are well adapted to this task. Using robot vision, autonomous
kitting robots identify individual products and assemble the assortment.


The robot system consists of a stationary overhead camera
and 7-DOF Panda Robotic Arm acting as the manipulator,
which can pick and place various objects as needed. The
objective of the system is to use color as a criteria for
kitting where each kit consists of a known assortment of
colored objects. Having identified the items to kit, the robot
picks the object up sequentially and moves them into their
respective bins. The primary application for color based kitting
is for candies and sweets kitted based on holidays, but this
application can be generalized further to allow implementation
in various industries to automated their assembly chain.

<p align="left" float="middle">
  <img width="50.0%" src="https://user-images.githubusercontent.com/31438120/207800575-0de66345-c470-4591-95f7-3b7eff4bc91e.gif"alt="Animation of robot arm working"/>
</p>

## Authors
---
### Phase 1
- **Driver** : Manu Madhu Pillai (117817928)
- **Navigator** : Akash Ravindra (117422085)

### Phase 2
- **Driver** : Akash Ravindra (117422085)
- **Navigator** : Manu Madhu Pillai (117817928)

### Phase 3
- **Driver** : Manu Madhu Pillai (117817928)
- **Navigator** : Akash Ravindra (117422085)

## Agile Iterative Process (AIP)
---
This project will be completed using AIP with the involvement of 2 programmers using Pair-programming in turns. The detailed Product Backlog, Iteration Backlogs and Work Log are mentioned in the link given below :


## Links
---
- [Agile Iterative Process Product Backlog, Sprint Backlog and Work Log](https://docs.google.com/spreadsheets/d/1y0EOs_R8pSBonxlyjeu_ELVRdtSt3HaSWdxoQPHqVec/edit?usp=sharing)

### Phase 0 (Proposal)

- [Project Proposal](/project_docs/phase_0/project_proposal.pdf)

### Phase 1 (Plan and Design)

- [Sprint 1 Notes](https://docs.google.com/document/d/1CZGV1a1UL9klj6oX2hPxPKNyNd7Ux1Ep_GPSj76Kjng/edit?usp=sharing)

- [UML Activity Diagram](/project_docs/phase_1/UML%20Activity%20Diagram.png)

- [UML Class Diagram](/UML/initial/UML%20diagram.png)

### Phase 2 (Implementation)
- [Sprint 2 Notes](https://docs.google.com/document/d/12m7j60CVa9yK7Lz7AEWvJMyVStsDL-hOpDRUbwhlt0A/edit#heading=h.27bxryqx38f0)

- [Quad Chart](https://docs.google.com/presentation/d/1bDqmEpYKWgdXigkWLl-adsmr33F7crr1vSKZkL6HYuA/edit?usp=sharing)

- [UML Class Diagram](/UML/revised/uml.jpeg)

### Phase 3 (Final Phase)
- [Sprint 3 Notes](https://docs.google.com/document/d/1aLDwV21oFEbNVUyM_sfGerGBF4CHGqHE5IiwTB9CbBY/edit?usp=sharing)

- [Presentation Slides](https://docs.google.com/presentation/d/1uNn4hR_tEWcC7nWF5dTGCcLEECf9hzVEJbslrFmw4jE/edit?usp=sharing)

- [Demonstration](https://drive.google.com/file/d/1UYEQATUw4-5YwZBkvKzuA7esgT25PIRq/view)

## Tools, libraries and dependencies used 
---
- C++ 14 
- ROS2 Humble
- OpenCV 
- CMake 
- Git 
- Github CI 
- Codecov
- RViz
- Gazebo
- Panda Manipulator
- MoveIt
- ROStest, GTest, GMock 

## UML Diagram
---
![image](/UML/revised/uml.jpeg)

## Build instructions
---

This package depends on ros2_control and gazebo_ros2_control. Due to the volatile state of ROS2 Humble, it is suggested that these packages be built from source to ensure that they are compatible with the package.

```bash
cd ~/ws/src
## Clone Repository
git clone git@github.com:lilnpuma/ProjectCollie.git
## Folder to  store the dependencies
mkdir depes
cd depes
## Clone the dependencies
git clone git@github.com:ros-controls/gazebo_ros2_control.git
git clone git@github.com:ros-controls/ros2_control.git
cd ros2_control
vcs import< ros2_control.humble.repos
cd ~/ws
## Update dependencies
rosdep update
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths src/
## Build the project
colcon build
## Source the project
. install/setup.bash
```

> :warning: During the the build if gazebo_ros2_control fails - The API for hardware info present in ros2_control has been updated and is due for an update in gazebo_ros2_control. Navigate to [gazebo_ros2_control_plugin.cpp](ENPM808x/ws/project_collie/src/depes/gazebo_ros2_control/gazebo_ros2_control/src/gazebo_ros2_control_plugin.cpp) and change the line 290 to the following 
```c++
std::string robot_hw_sim_type_str_ = control_hardware_info[i].hardware_plugin_name;
```


To build only the controller packages run the following build command
```bash
## Builds all the packages with the prefix panda ie; custom packages
colcon build  --packages-select-regex panda*
```

**Note:** stderr out for these packages (controller_manager, ros2controlcli, rqt_controller_manager) can be ignored. These are dependency packages using deprecated setup.py for installation instead of standards-absed tools like pip and build.

## Run Instructions
---
Make sure to complete building the packages succesfully as shown in the previous section before proceeding. 
```bash
cd ~/ws
## Source the project
. install/setup.bash
ros2 launch panda_controller controller.launch.py
```
The img stack and motion stack wait for 20 seconds after launch to start execution. This is to ensure that all objects have time to spawn in the Gazebo world.

## Test Instructions
---
### Build for Test Coverage

```bash
rm -rf build/panda_controller/
colcon build --cmake-args -DCOVERAGE=1 --packages-select panda_controller
cat log/latest_build/panda_controller/stdout_stderr.log
```
### How to run unit tests

```bash
source install/setup.bash
colcon test --packages-select panda_controller
cat log/latest_test/panda_controller/stdout_stderr.log
```

### How to generate code coverage report 
```bash 
source install/setup.bash
ros2 run panda_controller generate_coverage_report.bash
```
You can take a look at the generated report now.  For example:

```bash
firefox ~/ws/install/panda_controller/coverage/index.html
```
## Known Issues
---
- The spawned Panda robot behaves loses control sporadically. Cause not found yet.
- Gripper control is not optimal for picking object. Require a better mechanism.
- All functions could not be tested due to moveit package dependency.
- Rostests could not be done.
## License
---
This project is licensed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0). Further information can be found in the ```LICENSE``` file.
