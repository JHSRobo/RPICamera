#!/bin/bash

git pull
git reset --hard
rm -rf ../ROVMIND/ros_workspace/src/camera_viewer
mv camera_viewer ../ROVMIND/ros_workspace/src
echo "Move complete"