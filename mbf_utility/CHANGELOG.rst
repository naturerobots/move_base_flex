^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mbf_utility
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2026-02-10)
------------------
* Fixes tests, see #362

1.0.0 (2025-06-03)
------------------
* ROS 2 migration, see #346

0.4.0 (2021-10-26)
------------------
* return empty footprint if partly outside of the map, see #272

0.3.4 (2020-12-02)
------------------

0.3.3 (2020-11-05)
------------------
* fix controller fails if robot pose gets older than tf_timeout
* Make reference symbol position consistent across the project
* Move RobotInformation to mbf_utility, as it can be used generally

0.3.2 (2020-05-25)
------------------
* Remove dependency on base_local_planner and move FootprintHelper class to mbf_costmap_nav and make it static

0.3.1 (2020-04-07)
------------------

0.3.0 (2020-03-31)
------------------
* Add exception classes for get_path, exe_path and recovery
* unify license declaration to BSD-3

0.2.5 (2019-10-11)
------------------

0.2.4 (2019-06-16)
------------------
* Add check_point_cost service

0.2.3 (2018-11-14)
------------------
* Fix getRobotPose in melodic

0.2.2 (2018-10-10)
------------------

0.2.1 (2018-10-03)
------------------
* Make MBF melodic and indigo compatible
* Fix GoalHandle references bug in callbacks

0.2.0 (2018-09-11)
------------------
* Update copyright and 3-clause-BSD license

0.1.0 (2018-03-22)
------------------
* First release of move_base_flex for kinetic and lunar
