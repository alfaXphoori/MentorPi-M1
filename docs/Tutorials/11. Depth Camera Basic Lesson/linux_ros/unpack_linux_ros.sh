#!/bin/bash

printf '\xFD\x37\x7A\x58\x5A' | cat - linux_ros.pkg > linux_ros.tar.xz
tar -xJvf linux_ros.tar.xz
rm -rf linux_ros.tar.xz
cd linux_ros/
cp -rdf libs configurationfiles scripts linux/
cp -rdf libs configurationfiles scripts ros/src/ascamera/
cp -rdf libs configurationfiles scripts ros/src/as_nodelet/
cp -rdf libs configurationfiles scripts ros2/ascamera/
rm -rf libs/ scripts/ configurationfiles/ ../linux_ros.pkg
echo "unpack linux_ros package success"
