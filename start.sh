#!/bin/bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release  #编译
source install/setup.bash