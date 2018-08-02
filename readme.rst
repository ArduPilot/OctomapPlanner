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

Follow the steps here to install ardupilot_gazebo https://github.com/swiftgust/ardupilot_gazebo

Make sure to install gazebo-8 instead of gazebo-7 since visualization messages are not supported before gazebo-8

Users should first setup OSRF keys and install Gazebo 8 by executing

::

	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	sudo apt update
	sudo apt install libgazebo8-dev

.. tip::

ROS users can also install Gazebo 8 with ROS support after adding OSRF keys and using ros-kinetic-gazebo8-* packages

After installing Gazebo you can follow the instructions given here to setup your SITL environment https://github.com/swiftgust/ardupilot_gazebo

We will also need to install a few other dependencies

fcl

::

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

	git clone https://github.com/mavlink/mavlink.git
	cd mavlink
	git submodule update --init --recursive
	mkdir build
	cd build
	cmake ..
	sudo make install

OMPL

::

	wget http://ompl.kavrakilab.org/install-ompl-ubuntu.sh
	chmod u+x install-ompl-ubuntu.sh
	sh install-ompl-ubuntu.sh

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
This can be done by editing the planner_params.yaml file inside config folder

Finally, launch the planner code by executing this from the arduplanner folder

::

	./build/main_node