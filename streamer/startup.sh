#!/bin/bash

# Script to be run at EVERY boot

# Pull latest repo
cd /home/pi/Github/rpiCamera/
git pull

# Start up camera streamer
python3 /home/pi/Github/rpiCamera/streamer/streamer.py &
