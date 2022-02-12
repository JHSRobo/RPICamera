cd `find /home/pi -iname streamer || find /home/jhsrobo -iname streamer | head -1`
while true;
do
  git describe --tags --abbrev=0 | nc -k -l -p 80 -q 1
done
