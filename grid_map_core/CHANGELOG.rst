^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Improved efficiency for the Grid Map iterator (speed increase of 10x for large maps) ([#45](https://github.com/ethz-asl/grid_map/issues/45)).
* New iterator_benchmark demo to exemplify the usage of the iterators and their computational performance ([#45](https://github.com/ethz-asl/grid_map/issues/45)).
* Added new method to set the position of a grid map ([PR #42](https://github.com/ethz-asl/grid_map/pull/42)).
* Added new move_demo to illustrate the difference between the `move` and `setPosition` method.
* Fixed behavior of checkIfPositionWithinMap() in edge cases ([PR #41](https://github.com/ethz-asl/grid_map/pull/42)).
* Updated documentation for spiral and ellipse iterator, and iterator performance.
* const correctness for grid's getSubmap.
* Cleanup of arguments and return types.
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
* new ellipse iterator tool
* general improvements and bugfixes

1.0.0 (2015-11-20)
-------------------
* release for Springer ROS Book Chapter
