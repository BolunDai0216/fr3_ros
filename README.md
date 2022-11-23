# fr3_ros
ROS package for Franka Research 3 robot.

## Installation

This package relies on `libfranka`, `franka_ros`, `Pinocchio`, and `ProxSuite`. Both `libfranka` and `franka_ros` should be installed from scratch, for detail please refer to their [documentation](https://frankaemika.github.io/docs/installation_linux.html). The easiest way to install `Pinocchio` and `ProxSuite` is to use `conda`. After cloning this repo in the `src` folder in your `ROS` workspace, activate the conda environment that was just created to install the dependencies and run

```console
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```

Please deactivate the conda environment before running any of the launch files.