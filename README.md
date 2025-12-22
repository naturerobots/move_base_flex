[![Jazzy CI](https://github.com/naturerobots/move_base_flex/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/naturerobots/move_base_flex/actions/workflows/jazzy.yaml)
[![Humble CI](https://github.com/naturerobots/move_base_flex/actions/workflows/humble.yaml/badge.svg)](https://github.com/naturerobots/move_base_flex/actions/workflows/humble.yaml)

# Move Base Flex: A Highly Flexible Navigation Framework

Move Base Flex (MBF) is a modular and map-agnostic navigation framework for ROS that provides well-defined interfaces and action servers for path planning, control, and recovery behaviors. Rather than being a complete navigation stack, MBF serves as an adaptable interface layer that enables developers to design and integrate their own navigation systems using arbitrary map representations and custom plugin implementations. Key features are:

* **Map-Agnostic Interface Design**
  
  MBF's interfaces are independent of any particular map representation (e.g., [2D costmaps](https://github.com/amock/mbf_nav2_connector), [meshes](https://github.com/naturerobots/mesh_navigation), or voxel grids), enabling seamless integration, scientific comparison, and context-aware selection of both navigation implementations and map types.

* **Modular Action-Based Architecture**
  
  Separate action servers for *path planning*, *control*, and *recovery* enable external [deliberation software](https://github.com/ros-wg-delib/awesome-ros-deliberation) (e.g., Behavior Trees, SMACH, or custom logic) to coordinate complex navigation strategies.

* **Extensible Plugin Framework**
  
  Multiple planners, controllers, and recovery behaviors can be loaded simultaneously, selected at runtime, or executed in parallel using different concurrency slots.

* **Rich Feedback and Diagnostics**
  
  All actions expose detailed feedback, results, and error codes, providing transparent runtime information for better debugging and system supervision.

* **Clear Separation of Interfaces and Implementations**
  
  MBF's design facilitates reuse, experimentation, and the rapid development of new navigation approaches independent of any particular mapping or planning framework.

## Concepts & Architecture

Since robot navigation can be separated into planning and controlling in many cases, even for outdoor scenarios without the benefits of flat terrain, we designed MBF based on abstract planner-, controller- and recovery behavior-execution classes. To accomplish this goal, we created abstract base classes for the nav core BaseLocalPlanner, BaseGlobalPlanner and RecoveryBehavior plugin interfaces, extending the API to provide a rich and expressive interface. The abstract interfaces allow plugins to return valuable information in each execution cycle, e.g. why a valid plan or a velocity command could not be computed. This information is then passed to the external executive logic through MBF planning, navigation or recovering actions’ feedback and result. The planner, controller and recovery behavior execution is implemented in the abstract execution classes without binding the software implementation to 2D costmaps. In our framework, SimpleNav (or MeshNav) is just a particular implementation of a navigation system: its execution classes implement the abstract ones, bind the system to the costmaps. Thereby, the system can easily be used for other approaches, e.g. navigation on meshes or 3D occupancy grid maps. However, we provide a SimpleNavigationServer class without a binding to costmaps.

**MBF architecture:**
![MBF architecture](doc/images/move_base_flex.png)

## History

MBF was originally developed for ROS 1 by [Magazino](https://www.magazino.eu/en/) (see [noetic](https://github.com/naturerobots/move_base_flex/tree/noetic) or [master](https://github.com/naturerobots/move_base_flex/tree/master) branch) as a backwards-compatible replacement for `move_base`, providing a more flexible and transparent architecture when no modular alternative was available.
It has been successfully deployed in production environments, for example at [Magazino](https://www.magazino.eu/?lang=en), to control TORU robots operating in dynamic warehouse scenarios.

Compared to `move_base`, MBF introduced:

* Separate action servers for path planning, control, and recovery
* Detailed feedback and error reporting
* Runtime selection of multiple plugin implementations
* Map-agnostic interface definitions

With the advent of ROS 2 and newer navigation frameworks such as Nav2, MBF continues to serve as a lightweight, interface-oriented foundation for research, prototyping, and customized navigation systems.

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

## Further Resources

* [**Mesh Navigation**](https://github.com/naturerobots/mesh_navigation)
  
  Provides 3D navigation on mesh surfaces, implementing the MBF interfaces provided by this repository.

* [**MBF Nav2 Connector**](https://github.com/amock/mbf_nav2_connector)

  Provides 2D navigation on 2D costmaps by translating actions to nav2.

* [**MBF Deliberation Examples**](https://github.com/amock/mbf_deliberation)
  
  Demonstrates how to invoke MBF actions using popular deliberation frameworks such as BehaviorTree.CPP and SMACH. These examples work independently of the chosen map representation.


## Announcements & News

### 16.10.2024 First ROS2 Version of Move Base Flex
The first working ROS2 version of Move Base Flex has been published.
It targets the ROS2 distro `humble` and includes most components you know from ROS1:
- mbf_abstract_core & mbf_abstract_nav
- mbf_simple_core & mbf_simple_nav (for navigation components that need no map representation)
- mbf_utility
- mbf_msgs

The ROS2 version comes with an additional package that helps with integration tests:
- mbf_test_utility (only a test dependency)

These two packages not migrated:
- mbf_costmap_core & mbf_costmap_nav (for navigation components that utilize a 2D costmap). Nav2, which hosts the 2D costmap equivalent to the one from ROS1, and ROS1's move_base are quite different, so interfaces do not easily fit anymore. This makes migration hard. PRs are welcome for this. However, we might integrate another 2D grid map planning module soon.

Note that [mesh_navigation](https://github.com/naturerobots/mesh_navigation) is also available for ROS2, now. It provides navigation components that utilize 3D mesh maps.
