^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Separated OpenCV to grid map conversions to grid_map_cv package.
* Improved efficiency and generalized image to grid map conversion.
* Added image conversion for different encodings and depth resolutions.
* Fix scaling of image value to height conversion.
* Improved efficiency of the grid map to point cloud conversion by omitting invalid cells.
* Contributors: Peter Fankhauser, Daniel Stonier, Martin Wermelinger, Dominic Jud

1.2.0 (2016-03-03)
------------------
* Changed the package name from `grid_map` to `grid_map_ros` and made `grid_map` a metapackage (`#34 <https://github.com/ethz-asl/grid_map/issues/34>`_).
* Added new occupancy grid to grid map converter (`#33 <https://github.com/ethz-asl/grid_map/issues/33>`_).
* Contributors: Peter Fankhauser

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
* new conversion from grid map to image
* general improvements and bugfixes

1.0.0 (2015-11-20)
-------------------
* release for Springer ROS Book Chapter
