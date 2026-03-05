^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mbf_abstract_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2026-03-05)
------------------

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
* Fixes planner execution API compatibility, see #279 and https://github.com/uos/mesh_navigation/issues/18
* return empty footprint if partly outside of the map, see #272
* Allow the controller to handle cancel if properly implemented, see #274
* Do not reset cancel\_, as that will invalidate ongoing canceling
* Run replanning in its own thread, fixes #223, see #267
* enable max retries logic for controller execution test, see #264
* fix controller termination, see #263
* abstract controller execution unit tests, see #261
* planner action unit tests, see #244
* Fix number of retries check, see #262
* transform pose to global frame before calling planner in costmap planner execution, see #256
* Use ROS rates for sleeping to support simulation time, see #257
* wait for timeout in abstract eecution base, since cancel call could be missed, see #255
* add unit test for the abstract execution base, see #249
* fix race-condition in the test, see #254
* AbstractPlannerExecution refactoring and unit tests, see #237
* add documentation, missing includes, joinable check, and fixup and test the abstract_action_base

0.3.4 (2020-12-02)
------------------

0.3.3 (2020-11-05)
------------------
* On move_base action, handle properly RECALLED, REJECTED and LOST status, see #228
* Fill recovery result field used_plugin, see #229
* Signal from setState function, see #236
* Controller fails if robot pose gets older than tf_timeout, see #231
* Send move_base result when canceled during exe_path and recovery, see #218
* On controller cancel, wait for the control loop to stop, see #215
* Ensure MBF does not crash upon receiving an empty path, see #214
* Make robot_info a reference to keep one single instance, see #204
* mbf_abstract_nav action event logging from INFO to DEBUG, see #203
* Fix deallocation on shutdown by discarding all plugins and resetting action server pointers, see #199
* Move RobotInformation to mbf_utility, as it can be used generaly, see #196

0.3.2 (2020-05-25)
------------------
* Avoid duplicated warn logging output when we cannot cancel a plugin
* Remove unused methods and attributes from AbstractNavigationServer, which are already present at other places
* Reuse execution slots; cleanup only at destruction
* Enable different goal tolerance values for each action

0.3.1 (2020-04-07)
------------------

0.3.0 (2020-03-31)
------------------
* Clean up patience exceeded method
* Add last valid cmd time as class variable
* Add started state and improve output messages
* Unify license declaration to BSD-3
* Add parameter force_stop_on_cancel to send a zero-speed command on cancelation (default: true)
* remove explicit boost-exception dependency, Boost >= 1.69 provides exception by default.
* Allow the user time-consuming cancel implementations
* Rename abstract_action.h as abstract_action_base.hpp
* Remane robot_Information.cpp as robot_information.cpp
* Unify headers definitions and namespace intentation
* Add parameter to actively stop once the goal is reached
* Exit immediately from action done callbacks when action_state is CANCELED

0.2.5 (2019-10-11)
------------------
* Update goal pose on replanning, so the feedback remains consistent
* Fix: Reset oscillation timer after executing a recovery behavior
* Remove debug log messages
* Do not pass boost functions to abstract server to (de)activate costmaps.
  Run instead abstract methods (possibly) overridden in the costmap server,
  all costmap-related handling refactored to a new CostmapWrapper class
* On controller execution, check that local costmap is current
* On move_base action, use MoveBaseResult constant to fill outcome in case of oscilation

0.2.4 (2019-06-16)
------------------
* Reduce log verbosity by combining lines and using more DEBUG
* Concurrency container refactoring
* Prevent LOST goals when replanning
* Set as canceled, when goals are preempted by a new plan
* move setAccepted to abstract action
* moved listener notification down after setVelocity
* fix: Correctly fill in the ExePathResult fields
* Fix controller_patience when controller_max_retries is -1
* Change current_twist for last_cmd_vel on exe_path/feedback
* Replace recursive mutexes with normal ones when not needed
* Give feedback with outcome and message for success and error cases from the plugin.

0.2.3 (2018-11-14)
------------------
* Do not publish path from MBF
* Single publisher for controller execution objects
* Ignore max_retries if value is negative and patience if 0
* Avoid annoying INFO log msg on recovery

0.2.2 (2018-10-10)
------------------
* Add outcome and message to the action's feedback in ExePath and MoveBase

0.2.1 (2018-10-03)
------------------
* Fix memory leak
* Fix uninitialized value for cost
* Make MBF melodic and indigo compatible
* Fix GoalHandle references bug in callbacks

0.2.0 (2018-09-11)
------------------
* Update copyright and 3-clause-BSD license
* Concurrency for planners, controllers and recovery behaviors
* New class structure, allowing multiple executoin instances
* Fixes minor bugs

0.1.0 (2018-03-22)
------------------
* First release of move_base_flex for kinetic and lunar
