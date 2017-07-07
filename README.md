# Simulation package for Dyret
This repository contains the packages necessary for simulation Dyret. It
contains several packages that come together to model, interface and simulate.
Each package contains an individual `README` which describes further what each
package does, but a short summary is given below.

## Building
To build packages simply clone this repository into your
[workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and run
`catkin_make` or `catkin build`. Note that running `catkin_make -j1` can prevent
some difficulties in building these packages.

### Required packages
For beginners the easiest way to install ROS is by following
[this guide](http://wiki.ros.org/ROS/Installation).

Additionally to use Gazebo install the following:
```bash
$ sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs\
	ros-$ROS_DISTRO-ros-control\
	ros-$ROS_DISTRO-ros-controllers
```

## Package description
### `dyret_description`
This package contains [`URDF`](http://wiki.ros.org/urdf) which describes Dyret
so that it can be visualized in ROS/Rviz. This package is used by the other
to include a model of Dyret. For simplicity this package contains a handy launch
file that can be used to place the `URDF` in ROS parameter `robot_description`.

### `dyret_gazebo`
This package contains further configuration to interface Dyret with
Gazebo and give ROS control inside Gazebo. This package requires both ROS and
Gazebo, as such it can not be used standalone with Gazebo. This package contains
many useful launch files that can be used to start Gazebo and spawn Dyret
inside.

### `dyret_sim_utils`
This package contains utilities that can be handy when simulating Dyret. The
package contains only headers which can be included in dependent projects.
