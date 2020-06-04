#!/usr/bin/env python
#sudo apt-get install python-pil.imagetk
import cv2
import rospy
from Tkinter import *
from PIL import ImageTk, Image
from geometry_msgs.msg import Twist

cap_d=rospy.get_param("~video_capture","/dev/video0")

pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rospy.init_node('teleop_twist_keyboard',anonymous=True)

max_spd=.3 # used for angular and linear make the biggest the cmd_vel node will limit it anyways
scale_factor=.2 #by how much to increase the speed

fullscreen=True
root = Tk()
app = Frame(root, bg="white")
app.grid()
lmain = Label(app)
lmain.grid()

cap = cv2.VideoCapture(cap_d)

# function for video streaming
def video_stream():
    ret,frame=cap.read()
    if not ret:
        raise IOError("Can't open Video")
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(1, video_stream)
 

last_call=None
speed=0

def l(event):
    spd=getSpd("l") 
    pub_twist_from_spd(speed,(0,0,0,-1))
    
    
def r(event):
    spd=getSpd("r")        
    twist=pub_twist_from_spd(spd,(0,0,0,1))
 

def f(event):
    spd=getSpd("f") 
    twist=pub_twist_from_spd(speed,(1,0,0,0))
    
def b(event):
    spd=getSpd("b")        
    twist=pub_twist_from_spd(spd,(-1,0,0,1))    
 
def pub_twist_from_spd(speed,cords):
    #inspired by https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
    twist = Twist()
    x,y,z,th=cords
    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*speed
    pub.publish(twist)
    return twist


def getSpd(call):
    global speed
    global last_call
    if last_call==call:
        if speed<max_spd:
            speed+=scale_factor
    else:
        speed=scale_factor
    last_call=call

    return speed

def stop(event=None):
    global speed
    speed=0
    twist=pub_twist_from_spd(speed,(0,0,0,0))


app.focus()
root.bind('<Left>', l) #https://stackoverflow.com/questions/50494824/how-to-add-key-binding
root.bind('<Up>', f)
root.bind('<Right>', r)
root.bind('s',stop)
root.bind('<Down>', b)

if fullscreen:
    root.overrideredirect(True)
    root.overrideredirect(False)
    root.attributes('-fullscreen',True)


T = Text(root, height=100, width=100)
T.grid()
T.insert(END, """
LEGEND

Use arrow keys for navigation
Press S to stop

       ^
    <      >
       \/

""")

def onshutdown():
    stop()
    cap.release()
    try:
        root.destroy()
    except: #cause it was destroyed by sth else earlier
        pass

rospy.on_shutdown(onshutdown)
video_stream()
root.mainloop()