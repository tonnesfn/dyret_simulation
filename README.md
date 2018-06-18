# Simulation package for `DyRET`
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This repository contains the packages necessary for simulating `DyRET`. It
contains several packages that come together to model, interface and simulate. A
description of each package is given below, for further information look at
launch files and [`dyret_common`](https://github.com/dyret-robot/dyret_common).

## Documentation
For information about `DyRET` or how to use the simulation packages see our
[wiki](https://github.com/dyret-robot/dyret_documentation/wiki).

## Building
To build packages simply clone this repository into your
[workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace) and run
`catkin_make` or `catkin build`. Note that running `catkin_make -j1` can prevent
some difficulties in building packages that depend on each other.

### Required packages
Like all `DyRET` packages this requires
[`dyret_common`](https://github.com/dyret-robot/dyret_common).

## Package description
### `dyret_description`
This package contains [`URDF`](https://wiki.ros.org/urdf) which describes `DyRET`
so that it can be visualized in ROS/Rviz. This package is used by the other
to include a model of `DyRET`. For simplicity this package contains a handy launch
file that can be used to place the `URDF` in ROS parameter `robot_description`.

### `dyret_gazebo`
This package contains further configuration to interface `DyRET` with
Gazebo and give ROS control inside Gazebo. This package requires both ROS and
Gazebo, as such it can not be used standalone with Gazebo. This package contains
many useful launch files that can be used to start Gazebo and spawn `DyRET`
inside. Most users should utilize `empty_world.launch` to start everything
needed.

### `dyret_gazebo_plugin`
This package contains a Gazebo plugin that exposes an identical interface as the
real-world robot. Ideally this plugin should make the simulated robot behave as
the real.
