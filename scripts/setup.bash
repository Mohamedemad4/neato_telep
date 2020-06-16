#/usr/bin/bash
set -e

if [ -f '.expanded' ]; then
    echo "fs already expanded" 
    echo "Moving on ..."
        
else
    echo "resizing FS"
    touch .expanded
    wget -c http://files.pghnetwork.net/pghpunkid/fsExpandCheck_stretch.sh
    sudo sh fsExpandCheck_stretch.sh
    reboot
fi


sudo apt update
sudo apt install zsh git curl python-pip vim ffmpeg openvpn python-serial libangles-dev libbullet-dev liborocos-kdl1.3 libtf2-kdl-dev python-tf2-kdl liborocos-kdl-dev libeigen3-dev unzip -y
pip install Flask==1.0.2 gevent==1.4.0
su -c sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" pi

sudo tee /etc/rc.local > /dev/null <<EOT
#!/bin/sh -e
python /home/pi/telep_ws/src/scripts/mini_rest.py &
exit 0
EOT

cd /home/pi
#[ ! -d 'neato_telep' ] && git clone https://github.com/mohamedemad4/neato_telep
mkdir telep_ws
scp -r daruis1@192.168.1.3:~/repos/neato_telep/src telep_ws/
#cp -r neato_telep telep_ws/
cd telep_ws
#mv neato_telep src 
cd src
#
git submodule init
git submodule update
cd ..
source /home/pi/rosbots_catkin_ws/devel/setup.bash
catkin_make -j 2 # so the rpi doesn't crash due to low memory 
sudo chown -R pi:pi .
reboot