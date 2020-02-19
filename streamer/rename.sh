# rename the pi to camera and the last 2 digits of the mac address
MAC=$(cat /sys/class/net/eth0/address)

echo -e "127.0.0.1       localhost
::1             localhost ip6-localhost ip6-loopback
ff02::1         ip6-allnodes
ff02::2         ip6-allrouters

127.0.1.1      camera${MAC: -2}" > /etc/hosts

echo "camera${MAC: -2}" > /etc/hostname