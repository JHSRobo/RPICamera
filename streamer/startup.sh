#!/bin/bash

# Script to be run at EVERY boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

cd `find /home/pi -iname rpicamera | head -1` || exit

grep -q "Setup" /etc/rc.local || ( echo "Running setup" && bash setup.sh )

grep -q "Setup" /etc/rc.local || ( echo "Setup failed" && exit )

# Attempt to pull latest repo
git pull

# Start up camera streamer
bash raspivid.sh &
bash ping.sh &
