#!/bin/bash

export ROTATION=0
export WIDTH=1280
export HEIGHT=720

raspivid -n -cd MJPEG -ex fixedfps -ifx none -b 25000000 -br 60 -t 0 -rot ${ROTATION} -w ${WIDTH} -h ${HEIGHT} -fps 45 -o - | ncat -lkv4 5000

# -n to not show the video on the Raspberry Pi display
# -ih to insert H.264 headers into the stream
# -t 0 to keep streaming forever
# -rot ROTATION to rotate the stream
# -w WIDTH to set width
# -h HEIGHT to set heightx
# -fps FPS to set frames
# -b BITRATE to set bitrate
# -o to pass the output to STDOUT which allows it to be pushed to ncat

# -l 5000 to listen on port 5000
# -k to connect repeatedly
# -v to produce verbose error messages
# -4 to use IPv4 addresses only
