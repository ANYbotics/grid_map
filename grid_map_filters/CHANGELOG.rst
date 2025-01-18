^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.1 (2025-01-18)
------------------

2.2.0 (2024-07-23)
------------------
* Merge pull request `#458 <https://github.com/ANYbotics/grid_map/issues/458>`_ from ANYbotics/ci-temp-skip-octomap-server
  build: treat several build issues on rolling
* apply uncrustify except for EigenLab.hpp
* Merge pull request `#443 <https://github.com/ANYbotics/grid_map/issues/443>`_ from Ryanf55/update-maintainers
  Add Ryan as maintainer, remove Steve
* Add Ryan as maintainer, remove Steve
* Merge pull request `#419 <https://github.com/ANYbotics/grid_map/issues/419>`_ from Ryanf55/stop-using-CMAKE_COMPILER_IS_GNUCXX
  Stop using deprecated CMAKE_COMPILER_IS_GNUCXX
* Stop using deprecated CMAKE_COMPILER_IS_GNUCXX
  * Switch to CMAKE_CXX_COMPILER_ID
  * https://cmake.org/cmake/help/latest/variable/CMAKE_LANG_COMPILER_ID.html
* Contributors: Ryan, Ryan Friedman, wep21

2.1.0 (2022-11-08)
------------------

2.0.0 (2022-09-13)
------------------
* fix: mark Eigen library as SYSTEM
* fix: build error on jammy
* Initial ROS2 port
* Contributors: Maximilian Wulf, Steve Macenski

1.6.2 (2019-10-14)
------------------
* Update MinInRadiusFilter.cpp
* Contributors: Sascha Wirges

1.6.1 (2019-02-27)
------------------
* Updated host changes.
* Updated author e-mail address.
* Contributors: Peter Fankhauser, Péter Fankhauser

1.6.0 (2017-11-24)
------------------
* Implemented a set of new filters:
* Math expression filter based on EigenLab
* Sliding window math expression filter
* Normal vector and curvature filter
* Color map, normal color, light intensity filter, added color blending mode
* BufferNormalizerFilter
* Contributors: Péter Fankhauser, Tanja Baumann

1.5.2 (2017-07-25)
------------------

1.5.1 (2017-07-25)
------------------

1.5.0 (2017-07-18)
------------------
* Smaller cleanups for consistency.
* Contributors: Peter Fankhauser

1.4.2 (2017-01-24)
------------------

1.4.1 (2016-10-23)
------------------

1.4.0 (2016-08-22)
------------------

1.3.3 (2016-05-10)
------------------
* Release for ROS Kinetic.
* Contributors: Peter Fankhauser

1.3.2 (2016-05-10)
------------------

1.3.1 (2016-05-10)
------------------

1.3.0 (2016-04-26)
------------------

1.2.0 (2016-03-03)
------------------

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
* general improvements and bugfixes

1.0.0 (2015-11-20)
-------------------
* release for Springer ROS Book Chapter
