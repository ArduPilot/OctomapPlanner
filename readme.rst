.. _arduplanner-gazebo-sitl:

============
ArduPlanner
============

This article explains how to set up and use ArduPlanner to work with Gazebo SITL

Overview
===============

Arduplanner is a library for autonomous mapping and planning specifically designed for ArduPilot Copter. It uses Octomap for 3D occupancy mapping and OMPL and FCL for goal-directed planning with collision avoidance.

.. warning::

This library is still under development and has only been tested to work with Gazebo SITL for limited test cases. Further development and tests on hardware are still to be done.

Setup Instructions
==================

It is recommended to use Ubuntu 16.04 since all the tests have been conducted on it.

Make sure to install gazebo-8 instead of gazebo-7 since visualization messages are not supported before gazebo-8

Users should first setup OSRF keys and install Gazebo 8 by executing

::

	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	sudo apt update
	sudo apt install libgazebo8-dev

We will also use ros sources for installing other dependencies. To do that use the following commands

::

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	sudo apt update


.. tip::

ROS users can also install Gazebo 8 with ROS support after adding OSRF keys and using ros-kinetic-gazebo8-* packages

Follow the steps here to install ardupilot_gazebo https://github.com/swiftgust/ardupilot_gazebo

Dependencies Installation
=========================

fcl

::

	git clone https://github.com/danfis/libccd
	cd libccd
	mkdir build && cd build
	cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
	make && sudo make install
	
	git clone https://github.com/flexible-collision-library/fcl
	cd fcl
	mkdir build
	cd build
	cmake ..
	sudo make install


octomap

::

	sudo apt install liboctomap-dev

mavlink

::

	sudo apt install ros-kinetic-mavlink

OMPL

::

	sudo apt install ros-kinetic-ompl

Finally, build ArduPlanner and copy the iris model for gazebo to find

::

	git clone https://github.com/ayushgaud/arduplanner
	cd arduplanner
	mkdir build
	cd build
	cmake ..
	make
	cp -r ../models/iris_with_camera ~/.gazebo/models/

Running the code
================

Launch Gazebo with a demo world by executing 

::

	gazebo --verbose worlds/iris_gas_station_demo.world

On a seperate terminal start ArduCopter SITL

::

	sim_vehicle.py -v ArduCopter -f gazebo-iris

Before launching the code you may want to edit a few parameters like start and goal location
This can be done by editing the planner_params.yaml file inside the config folder

Finally, launch the planner code by executing this from the arduplanner folder

::

	./build/main_node