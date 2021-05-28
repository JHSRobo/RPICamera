#!/bin/bash

# Script to be run at EVERY boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

grep -q "Setup" /etc/rc.local || ( echo "Please run setup!" && exit )

# Start up camera streamer
bash `find /home/pi -iname raspivid.sh || find /home/jhsrobo -iname raspivid.sh | head -1` &
bash `find /home/pi -iname ping.sh || find /home/jhsrobo -iname ping.sh | head -1` &
