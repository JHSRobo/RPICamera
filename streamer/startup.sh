#!/bin/bash

# Script to be run at EVERY boot

# Attempt to pull latest repo
cd /home/camera/rpicamera/ || exit
git pull

# Start up camera streamer
python3 /home/camera/rpicamera/streamer/streamer.py &

exit 0
