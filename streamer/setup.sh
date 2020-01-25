#!/bin/bash

# Script to be run ONCE at the first boot

if [ "$(EUID)" == 0 ]
  then echo "Please run as root"
  exit
fi

echo -e "GET http://google.com HTTP/1.0\n\n" | nc google.com 80 > /dev/null 2>&1

if [ "$(echo -e "GET http://google.com HTTP/1.0\n\n" | nc google.com 80 > /dev/null 2>&1)" == 0 ]; then
    echo "Please connect to the internet to install PiCamera"
    exit
fi

apt update

dpkg -l | grep jq || apt install jq
dpkg -l | grep netcat || apt install netcat


# Safety checks
if [ "${USER}" == "jhsrobo" ]
then
  # main pi setup
  # doesn't rename the pi and doesn't restart it which should allow it to not break the whole thing

  # make startup executable
  chmod a+x /home/jhsrobo/rpicamera/streamer/startup.sh

  # edit rc.local
  echo -e "#!/bin/sh -e \nbash /home/jhsrobo/rpicamera/streamer/startup.sh &\nexit 0" > /etc/rc.local

  # reboot
  echo "Setup complete, please reboot"
  echo "#Setup" >> /etc/rc.local
  exit
else
  #setup for all of the camera modules

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
  echo "#Setup" >> /etc/rc.local
  reboot now
fi
