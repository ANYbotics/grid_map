^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_cv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

1.3.0 (2016-04-26)
------------------
* Separated OpenCV to grid map conversions to grid_map_cv package. The new methods
  are more generalized, faster, and can be used without ROS message types.
* Added new convenience function to change the resolution of grid maps with help of the OpenCV interpolation methods.
* Added `initializeFromImage()` to GridMapCvConverter.
* Added unit tests for grid_map_cv. Updated documentation.
* Resolving build errors for OpenCV.
* Fixed typo and added documentation.
* Contributors: Peter Fankhauser, Dominic Jud,
