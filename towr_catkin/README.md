# towr_catkin
Trajectory optimization for quadruped robots.

## 1. How to add a new robot

- create its KinematicModel and DynamicModel in models/examples.
- add your robot and robot's name in robot_model.h, and kinematics and dynamics in robot_model.cpp.
- add your robot's inverse kinematics in xpp, and change 'base2hip_LF_' data in inverse_kinematics_laikago.h.
- add your robot leg inverse kinematics.
- Instantiate your custom UrdfVisualizer() and run as ros node.
- Config rviz by load rviz node, it can change automatically when you save rviz config in the menu.
- add your robot's launch in all.launch.

## 2. Promble
- Robot state shows error rviz
  check robot tf, only load your robot description.

- tf parse incorrectly
  inverse kinematcis calculated incorrectly, check ee_pos_H variables.
