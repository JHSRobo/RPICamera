#!/bin/bash

# Script to be run at EVERY boot

# Start up camera streamer and restart it if it exits
while true; do
  python3 /home/jhsrobo/Github/rpicamera/streamer/streamer.py
  done

exit 0
