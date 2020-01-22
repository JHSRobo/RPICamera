# make startup executable
chmod a+x /home/camera/rpicamera/streamer/startup.sh

# edit rc.local
echo -e "#!/bin/sh -e \nbash /home/camera/rpicamera/streamer/startup.sh &\nexit 0" > /etc/rc.local

# reboot
reboot now
