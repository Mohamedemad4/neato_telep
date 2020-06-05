#/usr/bin/bash
set -e
sudo apt update
sudo apt install git x11vnc python-pil.imagetk python-tk -y
sudo apt-get install --no-install-recommends xserver-xorg -y
sudo apt-get install lxde-core lxappearance -y

sudo tee -a /etc/rc.local > /dev/null <<EOT
#!/bin/sh -e
nohup /home/pi/noVNC/utils/launch.sh --listen 6081 --vnc :5901 &
nohup x11vnc -display :0 -auth /var/run/lightdm/root/\:0 --loop --autoport 5901 --forever -geometry 1024x768  &
EOT

sudo adduser telep_user --gecos "First Last,RoomNumber,WorkPhone,HomePhone" --disabled-password
echo 123alpha | passwd telep_user --stdin

sudo su -c "echo '@/home/telep_user/autolaunch.sh' >> /home/telep_user/config/lxsession/LXDE/autostart "

cd /home/pi
git clone https://github.com/novnc/noVNC
git clone https://github.com/mohamedemad4/neato_telep

mkdir telep_ws
cp -r neato_telep telep_ws/
rm -rf neato_telep
cd telep_ws
mv neato_telep src 
cd src
sudo cp scripts/autolaunch.bash /home/telep_user
git submodule init
git submodule update
cd ..
source /home/pi/rosbots_catkin_ws/devel/setup.bash
catkin_make -j 2 # so the rpi doesn't crash due to low memory 