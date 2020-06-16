import os
import sys
import time
import threading
import requests as req
from orcha_utils import *
from orcha_logger import log
from neato_bot import neato_bot


class neato_pool():
    def __init__(self,mini_rest_port=7601,neato_ip_range="192.168.1.1/25",neato_ttl=300,houseKeep_every=10,hostname=None,rosmaster_uri=None):
        self.neatos={}
        self.mini_rest_port=mini_rest_port
        self.neato_ip_range=neato_ip_range
        self.houseKeep_every=houseKeep_every
        self.neato_ttl=neato_ttl
        self.hostname=hostname
        
        log.info("neato_ip_range={0};neato_ttl={1};mini_rest_port={2} ".format(self.neato_ip_range,self.neato_ttl,self.mini_rest_port))
        if not rosmaster_uri:
            try:
                self.rosmaster_uri=os.popen("echo $ROS_MASTER_URI").read()[0:-1]
            except:
                log.fatal("rosmaster_uri is not set! source ~/ros_ws/devel/setup.bash first!")
                sys.exit(1)
        try:
            if self.rosmaster_uri.split("://")[1].startswith("localhost"):
                log.fatal("rosmaster_uri must be accessable to other hosts not {0}".format(self.rosmaster_uri))
                sys.exit(1)
        except:
            log.fatal("rosmaster_uri {0} appears to be malformed ".format(self.rosmaster_uri))
            sys.exit(1)

        log.info("ROSMASTER is {0}".format(self.rosmaster_uri))
        try:
            if req.post(self.rosmaster_uri).status_code!=200:
                log.fatal("ROSMASTER doesn't appear to be a rosmaster. are you sure? it's correct")
                sys.exit(1)
        except:
            log.fatal("ROSMASTER doesn't appear to be running. Quitting...")
            sys.exit(1)

        
        if not hostname:
            log.fatal("Hostname isn't set! Quitting..")
            sys.exit(1)

        log.info("Hostname is {0}".format(hostname))

        log.info("Started pool Housekeeping Thread")
        self.pool_thread=threading.Thread(target=self.manage_neato_swarm)
        self.pool_thread.start()
    
    def manage_neato_swarm(self):
        while True:
            self.search_for_new_neatos()
            self.check_for_expired_neatos()
            time.sleep(self.houseKeep_every)
            log.debug("Housekeeping Swarm!")

    def search_for_new_neatos(self):
        for ip in scan_for_neatos(port=self.mini_rest_port,scan_range=self.neato_ip_range):
            if ip in [self.neatos[key].ip for key in self.neatos]:
                log.debug("Neato with ip:{0} is already in the pool, skipping..".format(ip))
                continue
            token=make_cool_name(dont_pick=self.getNeato_tokens())
            bot=neato_bot(ip=ip,token=token,rest_port=self.mini_rest_port,hostname=self.hostname,rosmaster_uri=self.rosmaster_uri)
            self.update_neatos(token,bot)

    def check_for_expired_neatos(self):
        for token in self.getNeato_tokens():
            bot=self.getNeato(token)
            if bot.been_logged_on_this_session==True:
                if (time.time()-bot.alive_since)>=self.neato_ttl:
                    log.info("expiring Neato with token:{0} and ip:{2} that has been alive_since:{1}".format(token,bot.alive_since,bot.ip))
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
        ip=getNeato(token).ip
        del self.neatos[token]
        log.info("Deleted Neato with Token:{0} and ip:{1} from pool".format(token,ip))
        return 

    def update_neatos(self,token,bot):
        log.info("Added New Neato with ip:{0} and token:{1}".format(bot.ip,token))
        self.neatos.update({token:bot})

