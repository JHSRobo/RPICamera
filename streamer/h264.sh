export ROTATION=0
export WIDTH=1280
export HEIGHT=720
export FPS=90
export PORT=5001

# load in environment variables from config.json
for output in $(jq -r 'keys[] as $k | "\($k)=\(.[$k])"' config.json)
do
  export "${output?}"
done

raspivid -n -cd MJPEG -ex fixedfps -ifx none -drc med -b 25000000 --intra 10 -lev 4 -br 60 -t 0 -rot $ROTATION -w $WIDTH -h $HEIGHT -fps $FPS -o - | nc -lkv4 $PORT

# -n to not show the video on the Raspberry Pi display
# -ih to insert H.264 headers into the stream
# -t 0 to keep streaming forever
# -rot ROTATION to rotate the stream
# -w WIDTH to set width
# -h HEIGHT to set heightx
# -fps FPS to set frames
# -b BITRATE to set bitrate
# -o to pass the output to STDOUT which allows it to be pushed to ncat

# -l 5001 listen on port 5001
# -k to connect repeatedly
# -v to produce verbose error messages
# -4 to use IPv4 addresses only
