#!/bin/bash

# Script to be run ONCE at the first boot

if [[ "$(id -u)" != 0 ]]
  then echo "Please run as root"
  exit
fi

if [[ "$(echo -e "GET http://google.com HTTP/1.0\n\n" | nc google.com 80 > /dev/null 2>&1)" == 0 ]]; then
    echo "Please connect to the internet to install libraries"
    exit
fi

apt update -y

dpkg -l | grep jq || apt install jq -y
dpkg -l | grep netcat || apt install netcat -y


# Safety checks
if [[ "${USER}" == "pi" ]]
then
  #setup for all of the camera modules

  # rename the pi to camera and the last 2 digits of the mac address
  MAC=$(cat /sys/class/net/eth0/address)

  echo -e "127.0.0.1       localhost
  ::1             localhost ip6-localhost ip6-loopback
  ff02::1         ip6-allnodes
  ff02::2         ip6-allrouters

  127.0.1.1      camera${MAC: -2}" > /etc/hosts

  echo "camera${MAC: -2}" > /etc/hostname

  # change the password of the pi
  useradd -m -d /home/camera/ -s /bin/bash -G sudo camera

  echo -e "JHSRobo\nJHSRobo" | passwd camera

  cd /home/camera || exit
  git clone https://github.com/jhsrobo/rpicamera

  # edit rc.local
  echo -e "#!/bin/bash -e \nbash /home/camera/rpicamera/streamer/startup.sh &\nexit 0" > /etc/rc.local

  # enable cameras
  raspi-config nonint do_camera 0

  # give more memory
  echo -e "$(sed '/gpu_mem/d' /boot/config.txt)" > /boot/config.txt
  echo "gpu_mem=356" >> /boot/config.txt

  # turn off the red light
  echo "disable_camera_led=1" >> /boot/config.txt

  # reboot
  echo "#Setup" >> /etc/rc.local
  reboot now
else
  # main pi setup
  # doesn't rename the pi and doesn't restart it which should allow it to not break the whole thing

  # edit rc.local
  echo -e "#!/bin/sh -e \nbash /home/jhsrobo/rpicamera/streamer/startup.sh &\nexit 0" > /etc/rc.local

  # reboot
  echo "Setup complete, please reboot"
  echo "#Setup" >> /etc/rc.local
  exit

fi
