^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2025-01-18)
------------------
* Merge pull request `#445 <https://github.com/Ryanf55/grid_map/issues/445>`_ from ANYbotics/mergify/bp/humble/pr-443
  Add Ryan as maintainer, remove Steve (backport `#443 <https://github.com/Ryanf55/grid_map/issues/443>`_)
* Add Ryan as maintainer, remove Steve
  (cherry picked from commit 852f67694637612e624e4c683a01ab589408b8e7)
* Merge pull request `#432 <https://github.com/Ryanf55/grid_map/issues/432>`_ from ANYbotics/mergify/bp/humble/pr-422
  update deprecated ogre header file (backport `#422 <https://github.com/Ryanf55/grid_map/issues/422>`_)
* update header file
  (cherry picked from commit a56647cc481ed7f2abe13b2ace5c1081cbd971d4)
* Contributors: JTaveau, Ryan, Ryan Friedman

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
* Made FrameManager and MessageFilterDisplay ROS version dependent.
* Update to tf2 MessageFilters
* Updated host changes.
* Updated author e-mail address.
* Contributors: Marco Sütterlin, Peter Fankhauser, Péter Fankhauser

1.6.0 (2017-11-24)
------------------
* Added option to rviz plugin to hide color faces.
* Fixed bug for RViz plugin.
* Fixing cpp-check warnings and errors.
* Contributors: Péter Fankhauser

1.5.2 (2017-07-25)
------------------
* Fixes error in grid_map_rviz_plugin when rendering lines.
* Contributors: Peter Fankhauser

1.5.1 (2017-07-25)
------------------

1.5.0 (2017-07-18)
------------------
* Improved visualization to support triangles in all directions.
* Fixes transparency issue for grid maps in RViz. (`#68 <https://github.com/anybotics/grid_map/issues/68>`_).
* Fixed package XML tag.
* Contributors: Perry Franklin, Stefan Fabian, Peter Fankhauser

1.4.2 (2017-01-24)
------------------
* Cleanup thanks to message traits.
* Contributors: Peter Fankhauser

1.4.1 (2016-10-23)
------------------
* Added functionality to display color from grid map layer.
* Added better handling of basic layers in Grid Map RViz plugin.
* Added functionality to invert rainbow colors in RViz plugin.
* Contributors: Philipp Kruesi, Péter Fankhauser

1.4.0 (2016-08-22)
------------------
* Added new package grid_map_rviz_plugin to visualize grid map layers as 3d surfaces.
* Updated documentation.
* Contributors: Péter Fankhauser, Philipp Kruesi
