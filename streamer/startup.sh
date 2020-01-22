#!/bin/bash

# Script to be run at EVERY boot

if [ "${USER}" == "camera${MAC: -2}" ]
then
  bash "/home/camera/rpicamera/streamer/setup.sh"
fi

# Attempt to pull latest repo
cd /home/camera/rpicamera/ || exit
#git checkout release
git pull

# Start up camera streamer and restart it if it exits
while true; do
  python3 /home/camera/rpicamera/streamer/streamer.py
  done

exit 0
