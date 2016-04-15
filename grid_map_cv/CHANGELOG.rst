^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_cv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added new convenience function to change the resolution of grid maps with help of the OpenCV interpolation methods.
* Added `initializeFromImage()` to GridMapCvConverter.
* Added unit tests for grid_map_cv. Updated documentation.
* Separated OpenCV to grid map conversions to grid_map_cv package. The new methods
  are more generalized, faster, and can be used without ROS message types.
* Contributors: Peter Fankhauser, Dominic Jud,
