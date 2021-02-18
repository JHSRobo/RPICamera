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

dpkg -l | grep ncat || apt install ncat -y
dpkg -l | grep nmap || apt install nmap -y


# Safety checks
if [[ -d "/home/pi" ]]
then
  # make sure the folder has the right file and edit rc.local
  (echo -en "#!/bin/bash -e \nbash "; echo -n "$(find "/home/pi" -iname "rpicamera/streamer/startup.sh" | head -1)"; echo -en " &\nexit 0") > /etc/rc.local 

  # change the password for the pi account
  echo -e "JHSRobo\nJHSRobo" | passwd pi

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
  # NOTE: REQUIRES THE FOLDER TO BE NAME rpicamera

  # edit rc.local
  echo -e "@reboot bash /home/jhsrobo/rpicamera/streamer/startup.sh" > /var/spool/cron/crontabs/root

  # reboot
  echo "Setup complete, please reboot"
  echo "#Setup" >> /etc/rc.local
  exit

fi
