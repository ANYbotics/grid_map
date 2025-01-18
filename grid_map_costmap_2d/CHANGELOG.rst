^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_costmap_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2025-01-18)
------------------
* Merge pull request `#449 <https://github.com/Ryanf55/grid_map/issues/449>`_ from ANYbotics/mergify/bp/humble/pr-424
  Use ament_export_targets in grid_map_costmap_2d (backport `#424 <https://github.com/Ryanf55/grid_map/issues/424>`_)
* Use ament_export_targets
  * Link to exported namespace targets when possible
  * And use INTERFACE targets
  (cherry picked from commit c0c7ef1e4da0bbe84a5ab4003e6429d0ecba66bf)
* Merge pull request `#445 <https://github.com/Ryanf55/grid_map/issues/445>`_ from ANYbotics/mergify/bp/humble/pr-443
  Add Ryan as maintainer, remove Steve (backport `#443 <https://github.com/Ryanf55/grid_map/issues/443>`_)
* Add Ryan as maintainer, remove Steve
  (cherry picked from commit 852f67694637612e624e4c683a01ab589408b8e7)
* Contributors: Ryan, Ryan Friedman

2.0.0 (2022-09-13)
------------------
* fix: mark Eigen library as SYSTEM
* Initial ROS2 port
* Contributors: Maximilian Wulf, Steve Macenski

1.6.2 (2019-10-14)
------------------

1.6.1 (2019-02-27)
------------------
* Merge pull request `#202 <https://github.com/ANYbotics/grid_map/issues/202>`_ from ANYbotics/fix/tf_for_indigo_and_kinetic
  Fix/tf for indigo and kinetic
* [grid_map_costmap_2d] Fixed test on melodic.
* [grid_map_costmap_2d] Tests using tf instead of tf2 for indigo/kinetic.
* Refactoring.
* Initializing ROSCostmap with tf buffer.
* typos fixed.
* fixed typo in initializer list.
* renamed member variable of costmap according to styleguide.
* fixed backwards compatibility for costmap_2d.
* Updated host changes.
* Updated author e-mail address.
* Contributors: Marco Sütterlin, Peter Fankhauser, Péter Fankhauser, Remo Diethelm, Samuel Bachmann

1.6.0 (2017-11-24)
------------------

1.5.2 (2017-07-25)
------------------

1.5.1 (2017-07-25)
------------------

1.5.0 (2017-07-18)
------------------
* Move costmap_2d conversion into separate package.
* Contributors: Peter Fankhauser
