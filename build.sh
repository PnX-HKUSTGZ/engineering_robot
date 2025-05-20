#!/bin/bash

# build interface package and robot_mdoel packages
colcon build --packages-select command_interfaces

# source setup.bash
source install/setup.bash

# build main package
colcon build