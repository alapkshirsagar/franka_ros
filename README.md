# ROS integration for Franka Emika research robots

[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

This repo expands the package by `franka_gazebo` and for now offers rudimentary
simulation functionality that also works with moveIt. For that a small action server
is running to provide the gripper action that moveIt uses but doesn't check for
stalling or max force.

**ATTENTION:** Because of the gravity bug in **gazebo7**, I switched to **gazebo9**. There
I had problems with the display of the models and the lighting. Therefore, I reconverted
the .dae models and added lights to the world as a workaround. I appreciate any suggestions
and fixes for any of the issues.

Some ideas/issues:
  - correct inertia values yet to insert
  - grasping of objects ([Jennifer Buehler's fix?](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin))
  - simulation of multiple robots in one environment (putting each one in its own namespace?)

To run the simulation including all necessary controllers, run `franka_gazebo.launch`.
To also start `the move_group` for planning purposes, run `panda_control_moveit_rviz`.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros
