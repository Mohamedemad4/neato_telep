## Setup instructions 
## For The Orcha Server

### For Production (SCRIPTS NOT READY FOR PRODUCTION YET CHECK OPEN ISSUES)
  - Launch A fresh VPS instance of Ubuntu 18.04 
  - SSH in and run ``` wget https://https://raw.githubusercontent.com/Mohamedemad4/neato_telep/master/scripts/setup_orcha.bash && sudo bash setup_orcha.bash```
  - Download the ```/root/neatos.ovpn``` file
  - you can Set Parameters And Configuration options in the ```/root/telep_ws/src/orcha/launch/orcha_init.launch``` for more info check ```orcha/neato_pool.py```
  - Reboot
  - **Before Following the Next Steps Make Sure you Have installed the Software on Each Robot First!**
  - go on the same network as the Neatos
  - clone this repo ```git clone https://github.com/mohamedemad4/neato_telep```
  - run ```python neato_telep/scripts/push_vpn_config.py --new_config /path/to/neatos.ovpn --neato_ip_range 192.168.1.1/25``` 
    - make sure to set the ```neato_ip_range``` to whatever range the neatos have in your local network (NOTE: MUST BE **Nmap** Compatible)
  - All Should be Set!

## For Each Robot
- download the [Rosbots raspberrypi image](https://github.com/ROSbots/rosbots_setup_tools)
- open a shell on the pi. make sure it's connected to the internet (here is a [handy tutorial](https://desertbot.io/blog/headless-raspberry-pi-3-bplus-ssh-wifi-setup) that should work for most pi models)
    - Username is **pi** password is **rosbots!**
- run ```wget https://https://raw.githubusercontent.com/Mohamedemad4/neato_telep/master/scripts/setup.bash && sudo bash setup.bash``` 
    - on the first run,the script will resize the filesystem then reboot **run it again** to install the system and all the packages

Enjoy!

## troubleshooting/common problems