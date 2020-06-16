import os
import rospy
import psutil
import string
import random
import requests as req
import subprocess as sp
from multiprocessing import Process

def scan_for_neatos(port=7061,scan_range="192.168.1.1/25"):
    a=os.popen("nmap -p {0} {1}".format(port,scan_range)).read() # TODO: use subprocess and check it it exits cleanly    
    open_hosts=[]
    for line_idx,line in enumerate(a.split("\n")):
        if line.startswith(str(port)) and line.split(" ")[1]=="open":
            possible_neato=a.split("\n")[line_idx-4].replace("(","").replace(")","").split(" ")[-1]
            try:
                verify_neato=req.get("http://{0}:{1}/make_me_coffee".format(possible_neato,port))
                if verify_neato.__dict__["status_code"]==418:
                    open_hosts.append(possible_neato)
                    rospy.logdebug("found host "+possible_neato)
            except Exception as e:
                rospy.logwarn(str(e))
    return open_hosts

def randomString(stringLength=4):
    letters = string.ascii_lowercase+string.ascii_uppercase
    return ''.join(random.choice(letters) for i in range(stringLength))

def make_cool_name(dont_pick=[]):    
    #credit to https://code.sololearn.com/cspLUgHPeAR6/#py
    first = ("Super", "Retarded", "Great", "Sexy", "Vegan", "Brave", 
    "Shy", "Cool", "Poor", "Rich", "Fast", "Gummy", "Yummy", "Masked", "Unusual", "American", "Bisexual", "MLG", "Mlg", "lil", "Lil")
    second = ("Coder", "Vegan", "Man", "Hacker", "Horse", "Bear", "Goat", 
    "Goblin", "Learner", "Killer", "Woman", "Programmer", "Spy", "Stalker", "Spooderman", "Carrot", "Goat", "Quickscoper", "Quickscoper")
    firrst = random.choice(first).lower()
    seccond = random.choice(second).lower()
    name = (firrst + "_" + seccond)
    dont_pick=[i.split("_")[0]+"_"+i.split("_")[1] for i in dont_pick if len(i.split("_"))==3]
    if name in dont_pick:
        return make_cool_name(dont_pick)
    return name+"_"+randomString()

def pick_an_open_port(port_range=[5000,10000]):
    "Picks an open port at random if it's being used pick another one,keep at it till it returns an unused port within port_range"
    open_ports=[i.laddr.port for i in psutil.net_connections()]
    proba_port=random.randint(port_range[0],port_range[1])
    if proba_port in open_ports:
        return pick_an_open_port(port_range=port_range)
    return proba_port

class session_pool():
    """
    Session_pool is used to run long running processes in the background
    Usage:

        sess_pool=session_pool()
    
        sess_pool.run_session_process("command")
        sess_pool.kill_session_process("command")

    NOTICE: supply command without the & argument run_session_process() handles pushing to the background for
    """
    def __init__(self):
        self.process_dict={}
        
    def session_process_thread(self,cmd):
        "Internal Method where the actual thread runs"
        rospy.logdebug("starting command "+cmd)
        exit_code=sp.call('({0})'.format(cmd),stdout=open(os.devnull, 'wb'),shell=True)
        return self.on_exit(cmd,exit_code)

    def run_session_process(self,cmd):   
        """run Session Process (nonblocking)"""
        proc_thread=Process(target=self.session_process_thread,args=(cmd,))
        proc_thread.start()
        self.process_dict.update({cmd:proc_thread})
        return proc_thread

    def kill_session_process(self,cmd):
        rospy.logdebug("killing {0} ".format(cmd))
        proc_thread=self.process_dict[cmd]
        proc_thread.terminate()
        del self.process_dict[cmd]

    def on_exit(self,cmd,exit_code):
        "Runs on command exit"
            if exit_code==0:
                rospy.logdebug("{0} exited with exit code 0".format(cmd))
            else:
                rospy.logwarn("{0} existed with NON zero exit code ".format(cmd))