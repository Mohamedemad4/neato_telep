#!/usr/bin/python2
import rospy
#rospy.init_node('neato_bot_listener',anonymous=True) # rospy needs to init_node first before we can init logging so rospy doesn't hijack our logger

import os
import time
from orcha_utils import *
from orcha_logger import log
from neato_bot import neato_bot
from neato_pool import neato_pool
from gevent.pywsgi import WSGIServer
from flask import Flask,Request,Response,render_template,request,abort

app = Flask(__name__)
app.debug = True
log.info("="*80+"\nStarted Neato Pool")
rospy.loginfo("Started Neato Pool")
bot_pool=neato_pool(hostname="192.168.1.3")



@app.route('/')
def hello():
    return "Wallah Under construction ha el 7een!"

@app.route('/login')
def login():
    "login into the neato redirects to  /play/token token is how you login"
    return "Wallah Under construction ha el 7een!"

@app.route("/play/<neato_token>")
def play(neato_token):
    "returns the page with the main JS elements that are used for streaming and playing"
    if neato_token not in bot_pool.getNeato_tokens():
        return "bot Expired",404

    bot=bot_pool.getNeato(neato_token)

    if bot.been_logged_on_this_session==False:
        bot.alive_since=time.time()
        bot.been_logged_on_this_session=True
    
    bot.start_streaming()
    bot.launch_telep()

    stream_url=bot.stream_url
    return "Wallah Under construction ha el 7een!"

@app.route("/debug/list_tokens/")
@app.route("/debug/list_tokens")
def debug_list_tokens():
    return "\n".join(i for i in bot_pool.getNeato_tokens())

@app.route("/send_cmd/<token>/<spd>/<x>/<y>/<z>/<th>/")
@app.route("/send_cmd/<token>/<spd>/<x>/<y>/<z>/<th>")
def forward_ctrl(token,spd,x,y,z,th):
    bot=bot_pool.getNeato(neato_token)
    if bot.been_logged_on_this_session==False: # say if a user has the token but chooses to access the endpoint directly without going through /play
        return "Get away from our APIs you Filthy Scoundrel!",401
    bot.pub_twist_from_spd(spd,(x,y,z,th))
    return ""

if __name__=="__main__":
    ph=("0.0.0.0",5000)
    rospy.loginfo("Starting Orcha Sever on "+str(ph))
    log.info("Starting Orcha Sever on "+str(ph))
    server = WSGIServer(ph, app) 
    server.serve_forever()