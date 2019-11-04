#!/bin/bash

# Script to be run ONCE at the first boot

# rename the pi to camera and the last 2 digits of the mac address
MAC=$(cat /sys/class/net/eth0/address)

echo -e "127.0.0.1       localhost \n\
::1             localhost ip6-localhost ip6-loopback \n\
ff02::1         ip6-allnodes\n\
ff02::2         ip6-allrouters\n\
\n\
127.0.1.1      camera${MAC: -2}" > /etc/hosts

echo "camera${MAC: -2}" > /etc/hostname

# make startup executable
chmod a+x /home/camera/rpicamera/streamer/startup.sh


# edit rc.local
echo -e "#!/bin/sh -e \n./home/camera/rpicamera/streamer/startup.sh &\nexit 0" > /etc/rc.local

# turn off the red light
echo "disable_camera_led=1" > /boot/config.txt

# reboot
reboot now
