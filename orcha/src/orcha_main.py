#!/usr/bin/python2
import os
import time
import rospy
from orcha_utils import *
from neato_bot import neato_bot
from neato_pool import neato_pool
from gevent.pywsgi import WSGIServer
from flask import Flask,Request,Response,render_template,request,abort

app = Flask(__name__)
app.debug = True

os.environ['ROSCONSOLE_FORMAT'] = '[${severity}:${time} -- ${file}:${line} -- ${function}()] ${message}'
rospy.init_node('neato_bot_listener',anonymous=True) # rospy needs to init_node first before we can init logging so rospy doesn't hijack our logger

mini_rest_port=rospy.get_param('~mini_rest_port', 7601)
neato_ip_range=rospy.get_param('~neato_ip_range', None)
neato_ttl=rospy.get_param('~neato_ttl', 300)
houseKeep_every=rospy.get_param('~houseKeep_every', 10)
stream_server_path=rospy.get_param('~stream_server_path', None)
hostname=rospy.get_param('~hostname', None)
rosmaster_uri=rospy.get_param('~rosmaster_uri', None)

bind_on=rospy.get_param('~bind_on', "0.0.0.0")
bind_port=rospy.get_param('~bind_port', 5000)

bot_pool=neato_pool(mini_rest_port=mini_rest_port,neato_ip_range=neato_ip_range,neato_ttl=neato_ttl
    ,houseKeep_every=houseKeep_every,stream_server_path=stream_server_path,hostname=hostname,rosmaster_uri=rosmaster_uri)

rospy.loginfo("Started Neato Pool")


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
    ph=(bind_on,bind_port)
    rospy.loginfo("Starting Orcha Sever on "+str(ph))
    server = WSGIServer(ph, app) 
    server.serve_forever()