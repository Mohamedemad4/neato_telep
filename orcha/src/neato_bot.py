import os
import time
import rospy
import requests as req
from orcha_utils import *
from geometry_msgs.msg import Twist

class neato_bot():
    def __init__(self,ip,token,rest_port=7061,streaming_port_range=[6000,10000],stream_server_path="./stream-server",hostname=None,rosmaster_uri=""):
        self.ip=ip
        self.token=token
        self.rest_port=rest_port
        self.rosmaster_uri=rosmaster_uri
        self.alive_since=0
        self.been_logged_on_this_session=False
        self.hostname=hostname
        self.streaming_port_range=streaming_port_range
        self.local_session_pool=session_pool()
        self.stream_server_path=stream_server_path
        self.cmd=""

        rospy.loginfo("New neato_bot() Created! rosmaster_uri:{0} ip:{1}".format(self.rosmaster_uri,self.ip))

    def start_streaming(self):
        "starts the stream-server and calls the neato to connect to it"
        ws_port=pick_an_open_port(self.streaming_port_range)
        ffmpeg_stream_port=pick_an_open_port(self.streaming_port_range)
        writebuffer_port,readbuffer_port=8192,8192 # accroding to netstat no one is using these port,so we might not need to assign them for each server? idk

        self.cmd="{stream_server_path} -incoming {ffmpeg_stream_port} -readbuffer {readbuffer_port} -secret {secret} -websocket {ws_port} -writebuffer {writebuffer_port}".format(
            ffmpeg_stream_port=ffmpeg_stream_port,ws_port=ws_port,writebuffer_port=writebuffer_port,readbuffer_port=readbuffer_port,secret=self.token,stream_server_path=self.stream_server_path)
        self.stream2url="http://{0}:{1}/{2}".format(self.hostname,ffmpeg_stream_port,self.token)
        self.stream_url="ws://{0}:{1}/".format(self.hostname,ws_port)
        
        rospy.loginfo("{0}:{1} is starting the stream at {2} from {3}".format(self.token,self.ip,self.stream_url,self.stream2url))

        self.local_session_pool.run_session_process(self.cmd)
        self.call_mini_rest_service("start_streaming/{0}".format(self.stream2url.replace("http://","").replace("/","-")))

        return

    def stop_streaming(self):
        "kills the stream-server and calls the neato to stop streaming"
        if self.cmd=="":
            rospy.logerror("call start_streaming First!")
            return 
        self.local_session_pool.kill_session_process(self.cmd)
        self.call_mini_rest_service("stop_streaming/{0}".format(self.stream2url))
        
        rospy.loginfo("{0}:{1} has stopped  the stream at {2} from {3}".format(self.token,self.ip,self.stream_url,self.stream2url))
        
        self.cmd,self.stream2url,self.stream_url="","","" #reset everything

        return

    def launch_telep(self):
        "launches the telep_stuff/telep_neato.launch"
        self.pub_vel = rospy.Publisher('/neato_{0}/cmd_vel'.format(self.token), Twist, queue_size = 1)
        rospy.loginfo("Publishing to /neato_{0}/cmd_vel".format(self.token))

        self.call_mini_rest_service("launch_telep/{0}/{1}".format(self.rosmaster_uri,self.token))
        rospy.loginfo("Launching Telep Services on {0}:{1}".format(self.token,self.ip))
        return
    
    def kill_telep(self):
        "kills the telep_neato.launch on the neato"
        self.call_mini_rest_service("kill_telep/{0}/{1}".format(self.rosmaster_uri,self.token))
        self.pub_vel.unregister()
        self.pub_vel=None
        rospy.loginfo("killing Telep Services on {0}:{1}".format(self.token,self.ip))
    
        return

    def call_mini_rest_service(self,serv):
        url="http://{0}:{1}/{2}".format(self.ip,self.rest_port,serv.replace("http://",""))
        rospy.logdebug("calling mini Rest with URL {0}".format(url))
        try:
            request=req.get(url)
            if request.__dict__["status_code"]==200:
                rospy.logdebug("good request,server says {0}".format(request.content))

            if request.__dict__["status_code"]!=200:
                rospy.logwarn("BAD request,server says {0}".format(request.content))
            
        except Exception as e:
            rospy.logwarn(str(e))
        return

    def pub_twist_from_spd(self,speed,cords):
        twist = Twist()
        x,y,z,th=cords
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*speed
        self.pub_vel.publish(twist)
        return twist