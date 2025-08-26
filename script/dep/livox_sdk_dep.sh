#! /bin/bash

set -e
cd ./../../src/driver/livox_driver/src/Livox-SDK2
mkdir build && cd build
cmake ..
make -j12
sudo make install
