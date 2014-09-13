Grid Map
======================

Overview
---------------

This is a C++ library to manage two-dimensional grid maps with multiple data layers. It is designed with mobile robots in mind and typical usage including storing local data such as `elevation`, `variance`, `color`, `friction_coefficient`, `quality`, `surface_normal_x`, `surface_normal_y`, `surface_normal_z` etc. It is used in the [Robot-Centric Elevation Mapping](https://github.com/ethz-asl/elevation_mapping) framework.

The storage structure is implemented as two-dimensional circular buffer so the map can be moved efficiently while the data keeps its correspodance to a fixed reference frame. This library builds on the [Eigen] library and provides interfaces and message definitions for [ROS]. Currently, only float data can be stored in the grid map.

**Author: Péter Fankhauser, pfankhauser@ethz.ch<br />
Affiliation: Autonomous Systems Lab, ETH Zurich**

![Grid map example in Rviz](example.jpg)


Citing
---------------

The methods used in this software are described in the following paper (available [here](http://dx.doi.org/10.3929/ethz-a-010173654)):

P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart,
**"Robot-Centric Elevation Mapping with Uncertainty Estimates"**,
in International Conference on Climbing and Walking Robots (CLAWAR), 2014.

    @inproceedings{Fankhauser2014RobotCentricElevationMapping,
      author = {Fankhauser, Péter and Bloesch, Michael and Gehring, Christian and Hutter, Marco and Siegwart, Roland},
      title = {Robot-Centric Elevation Mapping with Uncertainty Estimates},
      booktitle = {International Conference on Climbing and Walking Robots (CLAWAR)},
      year = {2014}
    }


Installation
------------

### Dependencies

- [Eigen]

### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_workspace/src
    git clone https://github.com/ethz-asl/elevation_mapping.git
    cd ../
    catkin_make


### Unit Tests

Run the unit tests with

    catkin_make run_tests_grid_map_lib run_tests_grid_map_lib

Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/grid_map/issues).


[ROS]: http://www.ros.org
[Eigen]: http://eigen.tuxfamily.org
