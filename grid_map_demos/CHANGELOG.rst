^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2025-01-18)
------------------
* Merge pull request `#445 <https://github.com/Ryanf55/grid_map/issues/445>`_ from ANYbotics/mergify/bp/humble/pr-443
  Add Ryan as maintainer, remove Steve (backport `#443 <https://github.com/Ryanf55/grid_map/issues/443>`_)
* Add Ryan as maintainer, remove Steve
  (cherry picked from commit 852f67694637612e624e4c683a01ab589408b8e7)
* Merge pull request `#427 <https://github.com/Ryanf55/grid_map/issues/427>`_ from ANYbotics/mergify/bp/humble/pr-419
  Stop using deprecated CMAKE_COMPILER_IS_GNUCXX (backport `#419 <https://github.com/Ryanf55/grid_map/issues/419>`_)
* Stop using deprecated CMAKE_COMPILER_IS_GNUCXX
  * Switch to CMAKE_CXX_COMPILER_ID
  * https://cmake.org/cmake/help/latest/variable/CMAKE_LANG_COMPILER_ID.html
  (cherry picked from commit 661aac270a5496cb334e53f25f10c036699e6801)
* Contributors: Ryan, Ryan Friedman

2.0.0 (2022-09-13)
------------------
* fix: mark Eigen library as SYSTEM
* fix: build error on jammy
* Initial ROS2 port
* Contributors: Maximilian Wulf, Steve Macenski

1.6.2 (2019-10-14)
------------------

1.6.1 (2019-02-27)
------------------
* Updated host changes.
* Updated author e-mail address.
* Contributors: Peter Fankhauser, Péter Fankhauser

1.6.0 (2017-11-24)
------------------
* Fixed compatibility issue with OpenCV 3 (`#140 <https://github.com/anybotics/grid_map/issues/140>`_).
* Fixing cpp-check warnings and errors.
* Fixed dependencies for grid_map_demos.
* Updated terrain for filters demo.
* Updated filter demo chain.
* Contributors: Perry Franklin, Péter Fankhauser

1.5.2 (2017-07-25)
------------------

1.5.1 (2017-07-25)
------------------

1.5.0 (2017-07-18)
------------------
* Added demo for octomap to grid map conversion.
* Updated installation for demos.
* General performance improvements, cleanups, and reformatting.
* Contributors: Jeff Delmerico, Peter Fankhauser

1.4.2 (2017-01-24)
------------------
* Addressing C++ compiler warnings.
* Contributors: Peter Fankhauser

1.4.1 (2016-10-23)
------------------

1.4.0 (2016-08-22)
------------------
* Added Grid Map RViz plugin to RViz configuration.
* Contributors: Peter Fankhauser

1.3.3 (2016-05-10)
------------------
* Release for ROS Kinetic.
* Contributors: Peter Fankhauser

1.3.2 (2016-05-10)
------------------
* Updated dependency to OpenCV for compatibility with ROS Kinetic and OpenCV 2/3.
* Contributors: Peter Fankhauser

1.3.1 (2016-05-10)
------------------
* Updated grid map loader demo.
* Contributors: Peter Fankhauser

1.3.0 (2016-04-26)
------------------
* Added new demonstrators for OpenCV based operations.
* Resolving build errors for OpenCV.
* Added comment on iterator performance (`#45 <https://github.com/anybotics/grid_map/issues/45>`_).
* Contributors: Peter Fankhauser

1.2.0 (2016-03-03)
------------------
* New iterator_benchmark demo to exemplify the usage of the iterators and their computational performance.
* Added new move_demo to illustrate the difference between the `move` and `setPosition` method.
* Contributors: Péter Fankhauser, Christos Zalidis, Daniel Stonier

1.1.3 (2016-01-11)
------------------

1.1.2 (2016-01-11)
------------------
* Should fix errors on build server regarding Eigen3 and visualization_msgs dependencies.

1.1.1 (2016-01-11)
------------------
* Changes to CMakeLists.txt to enable compatibility with Ubuntu Saucy.

1.1.0 (2016-01-08)
-------------------
* added installation instructions in CMakeLists
* updated demo for new ellipse iterator tool
* general improvements and bugfixes

1.0.0 (2015-11-20)
-------------------
* release for Springer ROS Book Chapter
