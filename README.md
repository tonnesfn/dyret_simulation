# Simulation package for `dyret`
This repository contains the packages necessary for simulating `dyret`. It
contains several packages that come together to model, interface and simulate. A
description of each package is given below, for further information look at
launch files and [`dyret_common`](https://github.uio.no/robin/dyret_common).

## Building
To build packages simply clone this repository into your
[workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace) and run
`catkin_make` or `catkin build`. Note that running `catkin_make -j1` can prevent
some difficulties in building these packages.

### Required packages
Like all `dyret` packages this requires
[`dyret_common`](https://github.uio.no/robin/dyret_common).

## Package description
### `dyret_description`
This package contains [`URDF`](https://wiki.ros.org/urdf) which describes `dyret`
so that it can be visualized in ROS/Rviz. This package is used by the other
to include a model of `dyret`. For simplicity this package contains a handy launch
file that can be used to place the `URDF` in ROS parameter `robot_description`.

### `dyret_gazebo`
This package contains further configuration to interface `dyret` with
Gazebo and give ROS control inside Gazebo. This package requires both ROS and
Gazebo, as such it can not be used standalone with Gazebo. This package contains
many useful launch files that can be used to start Gazebo and spawn `dyret`
inside. Most users should utilize `empty_world.launch` to start everything
needed.

### `dyret_gazebo_plugin`
This package contains a Gazebo plugin that exposes an identical interface as the
real-world robot. Ideally this plugin should make the simulated robot behave as
the real.
