#!/bin/bash

# Script to be run at EVERY boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

grep -q "Setup" /etc/rc.local || ( echo "Running setup" && bash setup.sh )

grep -q "Setup" /etc/rc.local || ( echo "Setup failed" && exit )

# Start up camera streamer
bash `find /home/pi -iname raspivid.sh | head -1` &
bash `find /home/pi -iname ping.sh | head -1` &
