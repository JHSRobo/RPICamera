while true
do
  wget --post-data "" 192.168.1.100:12345 -T 2 -q
  sleep 1
done