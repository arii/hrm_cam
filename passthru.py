import os
import cv2
import numpy as np
import pyfakewebcam
import rospy
from std_msgs.msg import String
import re

class HRMData:
    def __init__(self, topic="/hrm"):
        self.info = "not initialized"
        self.sub = rospy.Subscriber(topic, String, self.cb, queue_size=1)
    def getText(self):
        # TODO parse timestamp
        return self.info
    def cb(self, msg):
        self.info = msg.data
        print(self.info)

rospy.init_node("hrmview")
rospy.sleep(0.5)
hrmd = HRMData()



def write_text(image, text):
    # font 
    font = cv2.FONT_HERSHEY_SIMPLEX 
    # org 
    org = (50, 50) 
    
    # fontScale 
    fontScale = 2
       
    # Blue color in BGR 
    color = (255, 0, 0) 
         
    # Line thickness of 2 px 
    thickness = 2
            
    # Using cv2.putText() method 

    if "Heart rate:" in text:
        try:
            text = re.search('[1-9][0-9][0-9]?', text).group(0)
            fontScale = 10
            thickness = 10
            org = (100, 300)
        except:
            pass
    return cv2.putText(image, text, org, font,  fontScale, color, thickness, cv2.LINE_AA) 



# setup access to the *real* webcam

cap = cv2.VideoCapture('/dev/video0')

height, width = 720, 1280

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

cap.set(cv2.CAP_PROP_FPS, 60)


# setup the fake camera

fake = pyfakewebcam.FakeWebcam('/dev/video1', width, height)


# frames forever

while True:
    _, frame = cap.read()
    text=hrmd.getText()
    frame = write_text(frame, text)
    #frame = get_frame(cap, background_scaled)

    # fake webcam expects RGB

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    fake.schedule_frame(frame)
