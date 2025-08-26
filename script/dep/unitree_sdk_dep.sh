#! /bin/bash

set -e

git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
cmake --build build -j8
sudo cmake --build build -t install
cd ..
rm -rf unitree_sdk2
