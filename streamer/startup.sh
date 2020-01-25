#!/bin/bash

# Script to be run at EVERY boot
if [ -f "/home/camera" ]
then
   cd /home/camera/rpicamera || exit
else
  cd /home/jhsrobo/rpicamera || exit
fi

grep "Setup" /etc/rc.local || ( echo "Running setup" && bash setup.sh )

grep "Setup" /etc/rc.local || ( echo "Setup failed" && exit )

# Attempt to pull latest repo
#git checkout release
git pull

# Start up camera streamer and restart it if it exits
while true; do
  bash raspivid.sh
  done

exit 0
