## Setup instructions 
- download the [Rosbots raspberrypi image](https://github.com/ROSbots/rosbots_setup_tools)
- open a shell on the pi. make sure it's connected to the internet (here is a [handy tutorial](https://desertbot.io/blog/headless-raspberry-pi-3-bplus-ssh-wifi-setup) that should work for most pi models)
    - Username is **pi** password is **rosbots!**
- run ```wget https://https://raw.githubusercontent.com/Mohamedemad4/neato_telep/master/scripts/setup.bash && sudo bash setup.bash``` 
    - on the first run,the script will resize the filesystem then reboot **run it again** to install the system and all the packages
- change your ```/etc/hosts``` file to match the raspberrypi ip address [check here for why](#novnc_websocket_host_problem)
- open [http://raspberrypi:6081/vnc.html?host=raspberrypi&port=6081](http://raspberrypi:6081/vnc.html?host=raspberrypi&port=6081) in your browser
- enter the username **telep_user** and password **123alpha**

Enjoy!


## tasks
  - optimize local_rc_app to work smoothly on the raspberry pi (tested on pi 3b+)
  - test it with neato.py 
  - some webapp to manage users and sessions and whatnot (idk yet. probably needs a whole section)


## troubleshooting/common problems
 ### [NOVNC can't connect to server](#novnc_websocket_host_problem) 
   this usually because noVNC tries to connect to the websocket instance running on the hostname 
    for example if the PIs hostname is raspberrypi noVNC will try to connect ```ws://raspberrypi:6081``` even if you resolved the pi at ```http://anotherhostname:6081
