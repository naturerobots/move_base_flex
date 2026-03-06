^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mbf_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2026-03-06)
------------------

1.0.4 (2026-03-05)
------------------

1.0.3 (2026-03-05)
------------------
* Added action msgs for humble, see #371

1.0.2 (2026-02-10)
------------------
* Removed unused dependencies, see #368

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

0.3.2 (2020-05-25)
------------------
* add impassable outcome code for recovery behaviors
* enable different goal tolerance values for each action 

0.3.1 (2020-04-07)
------------------

0.3.0 (2020-03-31)
------------------
* add some more error codes, e.g. out of map, or map error
* unify license declaration to BSD-3

0.2.5 (2019-10-11)
------------------

0.2.4 (2019-06-16)
------------------
* Add check_point_cost service
* Change current_twist for last_cmd_vel on exe_path/feedback

0.2.3 (2018-11-14)
------------------
* Add outcome and message to the action's feedback in ExePath and MoveBase

0.2.1 (2018-10-03)
------------------

0.2.0 (2018-09-11)
------------------
* Concurrency for planners, controllers and recovery behaviors
* Adds concurrency slots to actions

0.1.0 (2018-03-22)
------------------
* First release of move_base_flex for kinetic and lunar
