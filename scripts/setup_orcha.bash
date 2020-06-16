#/usr/bin/bash
set -e

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
sudo apt install dnsutils python-pip nmap git ros-melodic-ros-base -y
pip install Flask==1.0.2 gevent==1.4.0 requests==2.22.0

HOSTNAME=$(dig @resolver1.opendns.com ANY myip.opendns.com +short)
echo $HOSTNAME

wget https://git.io/vpn -O openvpn-install.sh 
export APPROVE_INSTALL=y
export APPROVE_IP=y
export IPV6_SUPPORT=n
export PORT_CHOICE=1
export PROTOCOL_CHOICE=1
export DNS=1
export COMPRESSION_ENABLED=n
export CUSTOMIZE_ENC=n
export CLIENT=neatos
export PASS=1
./openvpn-install.sh

sudo tee /etc/openvpn/server/server.conf > /dev/null <<EOT
local $HOSTNAME
port 1194
proto tcp
dev tun
ca ca.crt
cert server.crt
key server.key
dh dh.pem
auth SHA512
tls-crypt tc.key
topology subnet
client-to-client 
duplicate-cn 
server 10.8.0.0 255.255.255.0
push "redirect-gateway def1 bypass-dhcp"
ifconfig-pool-persist ipp.txt
push "dhcp-option DNS 8.8.8.8"
push "dhcp-option DNS 8.8.4.4"
keepalive 10 120
cipher AES-256-CBC
user nobody
group nogroup
persist-key
persist-tun
status openvpn-status.log
verb 3
crl-verify crl.pem
EOT

sudo tee /etc/rc.local > /dev/null <<EOT
#!/bin/bash -e
sudo openvpn ~/neatos.ovpn &
source ~/telep_ws/devel/setup.bash
roslaunch orcha orcha_init.launch &
exit 0
EOT
sudo chmod +x /etc/rc.local

sudo systemctl enable openvpn-server@
sudo systemctl start openvpn-server@
cd ~/
[ ! -d 'neato_telep' ] && git clone https://github.com/mohamedemad4/neato_telep
mkdir telep_ws
cp -r neato_telep telep_ws/
cd telep_ws
mv neato_telep src 
cd src
git submodule init
git submodule update
cd ..
source /opt/ros/melodic/setup.bash
catkin_make -j 4