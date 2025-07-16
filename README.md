This branch contains the latest ROS1 (`noetic`) state. It is not longer being actively developed. For the latest state (ROS2), check out the [humble branch](https://github.com/naturerobots/move_base_flex/tree/humble).

# Move Base Flex: A Highly Flexible Navigation Framework


This repository contains Move Base Flex (MBF), a backwards-compatible replacement for move_base. MBF can use existing plugins for move_base, and provides an enhanced version of the same ROS interface. It exposes action servers for planning, controlling and recovering, providing detailed information of the current state and the plugin's feedback. An external executive logic can use MBF and its actions to perform smart and flexible navigation strategies. For example, at [Magazino](https://www.magazino.eu/?lang=en) we have successfully deployed MBF at customer facilities to control TORU robots in highly dynamical environments. Furthermore, MBF enables the use of other map representations, e.g. meshes. The core features are:
 
* Fully backwards-compatible with current ROS navigation.
* Actions for the submodules planning, controlling and recovering, and services to query the costmaps are provided. This interface allows external executives, e.g. SMACH, or Behavior Trees, to run highly flexible and complex navigation strategies.
* Comprehensive result and feedback information on all actions, including error codes and messages from the loaded plugins. For users still relying on a unique navigation interface, we have extended move_base action with detailed result and feedback information (though we still provide the current one).
* Separation between an abstract navigation framework and concrete implementations, allowing faster development of new applications, e.g. 3D navigation.
* Load multiple planners and controllers, selectable at runtime by setting one of the loaded plugin names in the action goal. 
* Concurrency: Parallel planning, recovering, controlling by selecting different concurrency slots when defining the action goal. Only different plugins instances can run in parallel.

Please see also the [Move Base Flex Documentation and Tutorials](https://wiki.ros.org/move_base_flex) in the ROS wiki. And [this repository](https://github.com/Rayman/turtlebot3_mbf) contains a working minimal configuration for a turtlebot 3.

## Concepts & Architecture

We have created Move Base Flex for a larger target group besides the standard developers and users of move_base and 2D navigation based on costmaps, as well as addressed move_base's limitations. Since robot navigation can be separated into planning and controlling in many cases, even for outdoor scenarios without the benefits of flat terrain, we designed MBF based on abstract planner-, controller- and recovery behavior-execution classes. To accomplish this goal, we created abstract base classes for the nav core BaseLocalPlanner, BaseGlobalPlanner and RecoveryBehavior plugin interfaces, extending the API to provide a richer and more expressive interface without breaking the current move_base plugin API. The new abstract interfaces allow plugins to return valuable information in each execution cycle, e.g. why a valid plan or a velocity command could not be computed. This information is then passed to the external executive logic through MBF planning, navigation or recovering actions’ feedback and result. The planner, controller and recovery behavior execution is implemented in the abstract execution classes without binding the software implementation to 2D costmaps. In our framework, MoveBase is just a particular implementation of a navigation system: its execution classes implement the abstract ones, bind the system to the costmaps. Thereby, the system can easily be used for other approaches, e.g. navigation on meshes or 3D occupancy grid maps. However, we provide a SimpleNavigationServer class without a binding to costmaps.

MBF architecture:
![MBF architecture](doc/images/move_base_flex.png)

## Future Work
MBF is an ongoing project. Some of the improvements that we have planned for the near future are:

* Release MBF Mesh Navigation, see [mesh_navigation](https://github.com/uos/mesh_navigation).
* Auto select the active controller when having concurrently running controllers
* Add Ackermann steering API
* Multi Goal API + Action
* Add new navigation server and core packages using [grid_map](https://wiki.ros.org/grid_map).
* Constraints-based goal (see issue https://github.com/naturerobots/move_base_flex/issues/8)

But, of course you are welcome to propose new fancy features and help make them a reality! Pull Requests are always welcome!

## Credit

### [<img width="25" height="25" src="doc/images/logos/magazino_icon.png"> Magazino](https://www.magazino.eu/) 
Move Base Flex was initially developed at Magazino.

### [<img width="25" height="25" src="doc/images/logos/nature_robots_icon.jpg"> Nature Robots](https://naturerobots.com/)
The latest version (ROS2) is developed and maintained by Nature Robots.


## Build Status

| ROS Distro  | GitHub CI | Develop | Documentation | Source Deb | Binary Deb |
|-------------|-----------|---------|---------------|------------|------------|
| **Noetic**  | [![Noetic CI](https://github.com/magazino/move_base_flex/workflows/Noetic%20CI/badge.svg)](https://github.com/magazino/move_base_flex/actions?query=workflow%3A%22Noetic+CI%22) | [![Build Dev Status](http://build.ros.org/buildStatus/icon?job=Ndev__move_base_flex__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__move_base_flex__ubuntu_focal_amd64) | [![Build Doc Status](http://build.ros.org/buildStatus/icon?job=Ndoc__move_base_flex__ubuntu_focal_amd64)](http://build.ros.org/job/Ndoc__move_base_flex__ubuntu_focal_amd64) | [![Build Src Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__move_base_flex__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__move_base_flex__ubuntu_focal__source) | [![Build Bin Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__move_base_flex__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__move_base_flex__ubuntu_focal_amd64__binary) | 
