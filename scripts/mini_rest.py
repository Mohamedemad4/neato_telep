#!/usr/bin/python2
import os
import logging
import functools
import subprocess as sp
from multiprocessing import Process
from gevent.pywsgi import WSGIServer
from flask import Flask,Request,Response,render_template,request,abort

FORMAT = "[%(asctime)s:%(levelname)s -- %(filename)s:%(lineno)s - %(funcName)s() ] %(message)s"
logging.basicConfig(filename='mini_rest.log',level=logging.DEBUG,format=FORMAT)
log=logging.getLogger('root')

res="640x480"
video_device="/dev/video0"

run_openvpn_cmd="openvpn orcha_vpn.ovpn"
ffmpeg_stream_cmd='ffmpeg -s {res} -f v4l2 -i {video_device} -f mpegts -codec:v mpeg1video -b:v 800k -r 24 -framerate 24 -codec:a mp2 -b:a 128k -muxdelay 0.001 {stream2URI}'
telep_neato_roslaunch_cmd='source /home/pi/telep_ws/devel/setup.sh && roslaunch group_ns:="neato_{0}" --server-uri {1} telep_stuff telep_neato.launch'
# app config
app = Flask(__name__)
app.debug = True


def log_requests_and_origin(f):
    #https://stackoverflow.com/questions/26736419/
    @functools.wraps(f)
    def decorated_function(*args, **kwargs):
        log.info("got a {1} request from {0}".format(request.remote_addr,request.full_path))
        return f(*args, **kwargs)
    return decorated_function

@app.route('/set_and_start_vpn_config/',methods=["POST"])
@app.route('/set_and_start_vpn_config',methods=["POST"])
@log_requests_and_origin
def start_vpn_service():
    new_vpn_conf=request.form["new_openvpn"]
    with open("orcha_vpn.ovpn","w+") as f:
        f.write(new_vpn_conf)

    if run_openvpn_cmd in sess_pool.process_dict.keys():
        sess_pool.kill_session_process(run_openvpn_cmd)
    sess_pool.run_session_process(run_openvpn_cmd)
    
    return "200ok!"

@app.route('/start_streaming/<stream2URI>')
@app.route('/start_streaming/<stream2URI>/')
@log_requests_and_origin
def start_streaming(stream2URI):
    stream2URI="http://"+stream2URI.replace("-","/")
    cmd=ffmpeg_stream_cmd.format(stream2URI=stream2URI,video_device=video_device,res=res)
    if cmd in sess_pool.process_dict.keys():
        return "process already running",400
    sess_pool.run_session_process(cmd)
    return "200ok!"

@app.route('/launch_telep/<rosmaster_URI>/<n>/')
@app.route('/launch_telep/<rosmaster_URI>/<n>')
@log_requests_and_origin
def launch_telep(rosmaster_URI,n):
    rosmaster_URI="http://"+rosmaster_URI
    cmd=telep_neato_roslaunch_cmd .format(n,rosmaster_URI)
    if cmd in sess_pool.process_dict.keys():
        return "process already running",400
    sess_pool.run_session_process(cmd)
    return "200ok!"


@app.route('/stop_streaming/<stream2URI>')
@app.route('/stop_streaming/<stream2URI>/')
@log_requests_and_origin
def stop_streaming(stream2URI):
    stream2URI="http://"+stream2URI.replace("-","/")
    cmd=ffmpeg_stream_cmd.format(stream2URI=stream2URI,video_device=video_device,res=res)
    if cmd not in sess_pool.process_dict.keys():
        return "no such process is running",400
    sess_pool.kill_session_process(cmd)
    return "200ok!"

@app.route('/kill_telep/<rosmaster_URI>/<n>/')
@app.route('/kill_telep/<rosmaster_URI>/<n>')
@log_requests_and_origin
def kill_telep(rosmaster_URI,n):
    rosmaster_URI="http://"+rosmaster_URI
    cmd=telep_neato_roslaunch_cmd.format(n,rosmaster_URI)
    if cmd not in sess_pool.process_dict.keys():
        return "no such process is running",400
    sess_pool.kill_session_process(cmd)
    return "200ok!"


@app.route("/make_me_coffee/")
@app.route("/make_me_coffee")
@log_requests_and_origin
def verify_mini_rest():
    return "I'm a teapot. Find a coffee maker.",418

class session_pool():
    def __init__(self):
        self.process_dict={}
        
    def session_process_thread(self,cmd):
        log.debug("starting command "+cmd)
        exit_code=sp.call('({0})'.format(cmd),stdout=open(os.devnull, 'wb'),shell=True,executable='/bin/bash')
        return self.on_exit(cmd,exit_code)

    def run_session_process(self,cmd):   
        proc_thread=Process(target=self.session_process_thread,args=(cmd,))
        proc_thread.start()
        self.process_dict.update({cmd:proc_thread})
        return proc_thread

    def kill_session_process(self,cmd):
        log.debug("killing {0} ".format(cmd))
        proc_thread=self.process_dict[cmd]
        proc_thread.terminate()
        del self.process_dict[cmd]

    def on_exit(self,cmd,exit_code):
            if exit_code==0:
                log.debug("{0} exited with exit code 0".format(cmd))
            else:
                log.warn("{0} existed with NON zero exit code ".format(cmd))
sess_pool=session_pool()
   
if __name__=="__main__":
    if os.path.exists("orcha_vpn.ovpn"):
        sess_pool.run_session_process(run_openvpn_cmd)
        log.info("Started Connecting to openvpn services")
    else:
        log.warn("Couldn't find orcha_vpn.ovpn not connecting to vpn on startup")
    server = WSGIServer(("0.0.0.0",7601), app) 
    server.serve_forever()
