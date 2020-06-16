import os
import sys
import time
import rospy
import threading
import requests as req
from orcha_utils import *
from neato_bot import neato_bot


class neato_pool():
    """
    neato_pool:Used to manage and interact with Neatos that are on the same network and running mini_rest.py
    Arguments:
        mini_rest_port: what port does mini_rest.py bind on?
        neato_ip_range: what's the Ip range of the Neatos? (MUST be readable by Nmap!) 
            ex: 192.168.1.1/25
        houseKeep_every: in seconds. how often to pool for new neatos and check for expired neatos
        neato_ttl: how long does a neato live after a user logs on on it? (check neato_bot.py for more details)
        
        stream_server_path: full path to stream-server binary file (taken from https://github.com/chanshik/jsmpeg-stream-go)

        hostname: what is the hostname of the Machine? (Used by neatos to access the server for streaming and such )
            ex: 
                - orcha.example.com
                - 10.8.0.1

        rosmaster_uri: a ROSMASTER_URI
            ex: http://10.8.0.1:11311 

        WARNING SECURITY NOTICE: the ROSMASTER port must NOT be exposed to the public service.

    """
    def __init__(self,mini_rest_port=7601,neato_ip_range="192.168.1.1/25",neato_ttl=300,houseKeep_every=10,stream_server_path="./stream-server",hostname=None,rosmaster_uri=None):
        self.neatos={}
        self.mini_rest_port=mini_rest_port
        self.neato_ip_range=neato_ip_range
        self.houseKeep_every=houseKeep_every
        self.neato_ttl=neato_ttl
        self.hostname=hostname
        self.stream_server_path=stream_server_path
        self.rosmaster_uri=rosmaster_uri

        rospy.loginfo("neato_ip_range={0};neato_ttl={1};mini_rest_port={2} ".format(self.neato_ip_range,self.neato_ttl,self.mini_rest_port))
        if not self.rosmaster_uri:
            try:
                self.rosmaster_uri=os.popen("echo $ROS_MASTER_URI").read()[0:-1]
            except:
                rospy.logfatal("rosmaster_uri is not set! source ~/ros_ws/devel/setup.bash first!")
                sys.exit(1)

        rospy.loginfo("ROSMASTER is {0}".format(self.rosmaster_uri))
        try:
            if req.post(self.rosmaster_uri).status_code!=200:
                rospy.logfatal("ROSMASTER doesn't appear to be a rosmaster. are you sure? it's correct")
                sys.exit(1)
        except:
            rospy.logfatal("ROSMASTER doesn't appear to be running. Quitting...")
            sys.exit(1)

        
        if not hostname:
            rospy.logfatal("Hostname isn't set! Quitting..")
            sys.exit(1)

        rospy.loginfo("Hostname is {0}".format(hostname))

        rospy.loginfo("Started pool Housekeeping Thread")
        self.pool_thread=threading.Thread(target=self.manage_neato_swarm)
        self.pool_thread.start()
    
    def manage_neato_swarm(self):
        "ongoing thread that does Housekeeping; calls search_for_new_neatos() and check_for_expired_neatos()"
        while True:
            self.search_for_new_neatos()
            self.check_for_expired_neatos()
            time.sleep(self.houseKeep_every)
            rospy.logdebug("Housekeeping Swarm!")

    def search_for_new_neatos(self):
        "Searches for New neatos on the IP range and adds them to the pool"
        for ip in scan_for_neatos(port=self.mini_rest_port,scan_range=self.neato_ip_range):
            if ip in [self.neatos[key].ip for key in self.neatos]:
                rospy.logdebug("Neato with ip:{0} is already in the pool, skipping..".format(ip))
                continue
            token=make_cool_name(dont_pick=self.getNeato_tokens())
            bot=neato_bot(ip=ip,token=token,rest_port=self.mini_rest_port,stream_server_path=self.stream_server_path,hostname=self.hostname,rosmaster_uri=self.rosmaster_uri)
            self.update_neatos(token,bot)

    def check_for_expired_neatos(self):
        "checks for neatos that have been used to expire them beyond their ttl"
        for token in self.getNeato_tokens():
            bot=self.getNeato(token)
            if bot.been_logged_on_this_session==True:
                if (time.time()-bot.alive_since)>=self.neato_ttl:
                    rospy.loginfo("expiring Neato with token:{0} and ip:{2} that has been alive_since:{1}".format(token,bot.alive_since,bot.ip))
                    self.expire_neato(token)

 
    def expire_neato(self,token):
        bot=self.getNeato(token)
        bot.stop_streaming()
        bot.kill_telep()

        new_token=make_cool_name(dont_pick=self.getNeato_tokens())
        bot.token=new_token
        bot.alive_since=0
        bot.been_logged_on_this_session=False

        self.del_neato(token)
        self.update_neatos(new_token,bot)

   
    def getNeato(self,token):
        return self.neatos[token]

    def getNeato_tokens(self):
        return [i for i in self.neatos.keys()]

    def del_neato(self,token):
        ip=self.getNeato(token).ip
        del self.neatos[token]
        rospy.loginfo("Deleted Neato with Token:{0} and ip:{1} from pool".format(token,ip))
        return 

    def update_neatos(self,token,bot):
        rospy.loginfo("Added New Neato with ip:{0} and token:{1}".format(bot.ip,token))
        self.neatos.update({token:bot})

