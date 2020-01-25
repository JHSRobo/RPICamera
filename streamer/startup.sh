#!/bin/bash

# Script to be run at EVERY boot
if test "/home/camera"
then
   cd /home/camera/rpicamera/streamer || exit
else
  cd /home/jhsrobo/rpicamera/streamer || exit
fi

grep -q "Setup" /etc/rc.local || ( echo "Running setup" && bash setup.sh )

grep -q "Setup" /etc/rc.local || ( echo "Setup failed" && exit )

# Attempt to pull latest repo
#git checkout release
git pull

# Start up camera streamer
bash raspivid.sh
