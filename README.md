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
  - simulation of multiple robots in one environment (putting each one in its own namespace?)

To run the simulation, launch `franka_gazebo franka_gazebo.launch` (in case you want to use the
velocity interface/controller, set the parameter `hw_interface:=VelocityJointInterface`).
This is pretty much equivalent to launching `franka_control franka_control.launch`.
Afterwards the **position_joint_trajectory_controller** or the **velocity_jointgroup_controller**
can be started to control the robot.
Alternatively, to start the **move_group** for planning purposes, instead of launching
the launch files mentioned above, just launch either `franka_gazebo panda_control_moveit_rviz.launch`
or `panda_moveit_config panda_control_moveit_rviz.launch`.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros
