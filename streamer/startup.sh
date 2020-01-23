#!/bin/bash

# Script to be run at EVERY boot
if [ -f "/home/camera" ]
then
   cd /home/camera/rpicamera || exit
else
  cd /home/jhsrobo/Github/rpicamera || exit
fi

grep "Setup" /etc/rc.local || ( echo "running setup" && bash setup.sh )

# Attempt to pull latest repo
#git checkout release
git pull

# Start up camera streamer and restart it if it exits
while true; do
  python3 streamer.py
  done

exit 0
