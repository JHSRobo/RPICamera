#!/bin/bash

# Script to be run at EVERY boot

# Attempt to pull latest repo
cd /home/pi/rpicamera/
git pull

# Start up camera streamer
python3 /home/pi/rpicamera/streamer/streamer.py &
