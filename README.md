# fr3_ros
ROS package for Franka Research 3 robot.

## Installation

This package relies on `libfranka`, `franka_ros`, `Pinocchio`, and `ProxSuite`. Both `libfranka` and `franka_ros` should be installed from source, for detail please refer to their [documentation](https://frankaemika.github.io/docs/installation_linux.html). The easiest way to install `Pinocchio` and `ProxSuite` is to use `conda`

```console
conda install proxsuite==0.2.7 -c conda-forge
conda install pinocchio -c conda-forge
```

After cloning this repo in the `src` folder in your `ROS` workspace, activate the conda environment that was just created to install the dependencies

```console
roscd && cd ../src
git clone https://github.com/BolunDai0216/fr3_ros.git
roscd && cd ..
conda activate name-of-conda-env
```

Then run

```console
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```

Please deactivate the conda environment before running any of the launch files.

```console
conda deactivate
```