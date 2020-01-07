#!/bin/bash

set -e # fail on error

cd /github/workspace

dpkg-buildpackage -b -us -uc
# source /opt/ros/kinetic/setup.bash
# rm -rf build #clean the build
# mkdir build
# cd build
# pwd
# cmake ..
# make
