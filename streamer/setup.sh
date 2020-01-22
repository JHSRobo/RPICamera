#!/bin/bash

# Script to be run ONCE at the first boot

# Safety checks
if [ "${USER}" == "jhsrobo" ]
then
  echo "This is not allowed for the main ROV image, please use main-pi-setup.sh"
  exit
fi

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

# install python3
dpkg -l | grep python3-picamera || apt install python3-picamera

dpkg -l | grep python3-picamera || (echo "Picamera library failed to install. Please install it manually using sudo apt install python3-picamera" && exit)

# rename the pi to camera and the last 2 digits of the mac address
MAC=$(cat /sys/class/net/eth0/address)

echo -e "127.0.0.1       localhost
::1             localhost ip6-localhost ip6-loopback
ff02::1         ip6-allnodes
ff02::2         ip6-allrouters

127.0.1.1      camera${MAC: -2}" > /etc/hosts

echo "camera${MAC: -2}" > /etc/hostname

# make startup executable
chmod a+x /home/camera/rpicamera/streamer/startup.sh

# edit rc.local
echo -e "#!/bin/sh -e \nbash /home/camera/rpicamera/streamer/startup.sh &\nexit 0" > /etc/rc.local

# enable cameras
sudo raspi-config nonint do_camera 0

# turn off the red light
echo "disable_camera_led=1" >> /boot/config.txt

# reboot
reboot now
