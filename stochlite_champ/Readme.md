# Stochlite Packages

* [stoch_linear](stoch_linear) is a core library that includes functions to perform walking based on [Robust Quadrupedal Locomotion on Sloped Terrains: A Linear Policy Approach](https://github.com/StochLab/SlopedTerrainLinearPolicy)
* [stochlite_base](stochlite_base) is a package that handles and calls all ros nodes to do walking in Gazebo (similar to [champ_base](../champ_base))
* [stochlite_config](stochlite_config) is a config package that contains all the parameters to tuning walking, eg. PID gains for ros_control
* [stochlite_description](stochlite_description) is a package that contains URDF files

## How to Install

*Note: At the moment, there are a few issues while running `catkin build`*

```
cd <path_to_your_workspace>
catkin_make
```

## How to Run the linear policy

```
source <path_to_your_workspace>/devel/setup.bash
roslaunch stochlite_config gazebo.launch 
```

For debugging purposes in RViz:
* In the file [bringup.launch](stochlite_config/launch/bringup.launch), Comment the line `<arg name="description_file"       default="$(find stochlite_description)/urdf/stochlite.urdf"/>` and Uncomment the line `<arg name="description_file"       default="$(find stochlite_description)/urdf/stochlite_fixed.urdf"/>`
    - The reason for doing so is due to some issues with fixed frame for moving robot. Hence using stochlite_fixed for onrack testing
* `roslaunch stochlite_config bringup.launch rviz:=true`