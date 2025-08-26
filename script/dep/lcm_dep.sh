#! /bin/bash

set -e

git clone https://github.com/lcm-proj/lcm.git
cd lcm
cmake -S . -B build
cmake --build build -j8
sudo cmake --build build -t install
cd ..
rm -rf lcm
