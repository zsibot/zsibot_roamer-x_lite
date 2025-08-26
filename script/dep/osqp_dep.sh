#! /bin/bash

set -e

git clone https://github.com/osqp/osqp
cd osqp
git checkout v0.5.0

git submodule update --init --recursive

mkdir -p build && cd build
cmake ..
make -j$(nproc)

sudo make install

cd ../..
rm -rf osqp
