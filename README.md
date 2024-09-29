# Admittance_Controller

## Overview

This package contains an admittance controller for a delta parallel manipulator.

**Keywords:** ROS, Melodic, Delta, Admittance Controller


### License

**Author: Victor Rosillo<br />
University of Malaga
Maintainer: Victor Rosillo Suero, vrosillo1110@gmail.com**

<!-- This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed. -->

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) -->

![Delta manipulator](images/Manipulador_delta.pdf)


### Publications

Bachelor's Thesis: Adaptive control of a lightweight three-degree-of-freedom parallel
manipulator

## Installation

### Installation from Packages
ROS version Melodic
    
Use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/Robotics-Mechatronics-UMA/admittance_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

Run the dummy test for the controller, you can save a rosbag file and watch the results of the simulation

	roslaunch admittance_controller controller.launch

## Config files

params.cfg: Config file for changing controller parameters in real time.

## Launch files

admittance_serial.launch: Launch file to perform real tests with a 3DoF force sensor. 

admittance_dummy.launch: Launch file to perform controller simulations using online force signals.


## Nodes

Node1 admittance_controller_node: An admittance controller for a 6 DoF parallel manipulator.

Node2 force_dummy_node: A test node that you can launch if you want to test the admittance controller and check the results without using a real force sensor. In other words, force will be applied online.

### ADMITTANCE_CONTROLLER

#### Subscribed Topics

* **`/Force`** ([geometry_msgs/Vector3])

	Topic where force sensor readings are published.


#### Published Topics

* **`/cmd_vel_ee`** ([geometry_msgs/Vector3])

	Topic where desired velocity of the end-efector is published.

## Bugs & Feature Requests

## Operating scheme

![ROS Interface](images/Diagrama_Comunicacion_ROS.pdf)


