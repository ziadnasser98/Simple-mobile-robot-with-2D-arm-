# Simple-mobile-robot-with-2D-manipulator-

## Overview
This repository contains a description of a simple differential mobile robot as urdf files. The robot has a 2-DOF revolute manipulator and a camera. Special plugins were implemented to allow the robot to control the robot's and the manipulator's motion. Finally, the potential field algorithm was used to make the robot reach a specified goal by the user. 
## Project structure
- my_robot_description package: this package contains the urdf files defining the robot.
- my_robot_bringup package: this package contains the launch files used in this project. There are three launch files:
  
  1- display.launch.py/.xml: to display the robot using Rviz and check the tfs of the robot.
  
  2- my_robot_gazebo.launch.xml: to view the robot in the Gazebo simulator where you can move it and manipulate the arm using plugins.
  
  3- potential_field_controller.launch.py: to make the robot reach a desired goal inside the Gazebo simulator using the potential field algorithm.
  
- my_robot_control package: this package contains the control algorithm of the robot. 
