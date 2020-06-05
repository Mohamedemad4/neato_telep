#/usr/bin/bash
set -e
sudo apt update
sudo apt install git x11vnc python-pil.imagetk python-tk unzip -y
sudo apt-get install --no-install-recommends xserver-xorg -y
sudo apt-get install lxde-core lxappearance -y

sudo tee /etc/rc.local > /dev/null <<EOT
#!/bin/sh -e
nohup /home/pi/noVNC/utils/launch.sh --listen 6081 --vnc :5901 &
nohup x11vnc -display :0 -auth /var/run/lightdm/root/\:0 --loop --autoport 5901 --forever -geometry 1024x768  &
exit 0
EOT
wget http://files.pghnetwork.net/pghpunkid/fsExpandCheck_stretch.sh
sudo sh fsExpandCheck_stretch.sh
sudo adduser telep_user --gecos "First Last,RoomNumber,WorkPhone,HomePhone" --disabled-password
echo 'telep_user:123alpha' | chpasswd
cd /home/telep_user
wget -c https://github.com/Mohamedemad4/neato_telep/raw/master/telep_user_config.zip
unzip telep_user_config.zip
cd /home/pi
[ ! -d 'noVNC' ] && git clone https://github.com/novnc/noVNC
[ ! -d 'neato_telep' ] && git clone https://github.com/mohamedemad4/neato_telep
mkdir telep_ws
cp -r neato_telep telep_ws/
cd telep_ws
mv neato_telep src 
cd src
sudo cp scripts/autolaunch.bash /home/telep_user
git submodule init
git submodule update
cd ..
source /home/pi/rosbots_catkin_ws/devel/setup.bash
catkin_make -j 2 # so the rpi doesn't crash due to low memory 
sudo chown -R pi:pi .