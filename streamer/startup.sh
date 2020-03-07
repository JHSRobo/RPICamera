#!/bin/bash

# Script to be run at EVERY boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

if [[ -f "/home/pi/rpicamera/streamer/raspivid.sh" ]]
then
   cd /home/pi/rpicamera/streamer || exit
else
  cd /home/jhsrobo/rpicamera/streamer || exit
fi

grep -q "Setup" /etc/rc.local || ( echo "Running setup" && bash setup.sh )

grep -q "Setup" /etc/rc.local || ( echo "Setup failed" && exit )

# Attempt to pull latest repo
#git checkout release

git pull

# Start up camera streamer
bash raspivid.sh &
bash ping.sh &
