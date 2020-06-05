#/usr/bin/bash
set -e
#autolaunch script that will launch everything once the user is connected 
source /home/pi/telep_ws/devel/setup.bash
roslaunch telep_stuff telep_neato.launch