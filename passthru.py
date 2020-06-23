import os
import cv2
import numpy as np
import pyfakewebcam
import rospy
from std_msgs.msg import String


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

def post_process_mask(mask):
    mask = cv2.dilate(mask, np.ones((10,10), np.uint8) , iterations=1)
    mask = cv2.blur(mask.astype(float), (30,30))
    return mask


def shift_image(img, dx, dy):
    img = np.roll(img, dy, axis=0)
    img = np.roll(img, dx, axis=1)

    if dy>0:

        img[:dy, :] = 0

    elif dy<0:

        img[dy:, :] = 0

    if dx>0:

        img[:, :dx] = 0

    elif dx<0:

        img[:, dx:] = 0

    return img


def hologram_effect(img):

    # add a blue tint

    holo = cv2.applyColorMap(img, cv2.COLORMAP_WINTER)

    # add a halftone effect

    bandLength, bandGap = 2, 3

    for y in range(holo.shape[0]):

        if y % (bandLength+bandGap) < bandLength:

            holo[y,:,:] = holo[y,:,:] * np.random.uniform(0.1, 0.3)

    # add some ghosting

    holo_blur = cv2.addWeighted(holo, 0.2, shift_image(holo.copy(), 5, 5), 0.8, 0)

    holo_blur = cv2.addWeighted(holo_blur, 0.4, shift_image(holo.copy(), -5, -5), 0.6, 0)

    # combine with the original color, oversaturated

    out = cv2.addWeighted(img, 0.5, holo_blur, 0.6, 0)

    return out

def write_text(image, text):
    # font 
    font = cv2.FONT_HERSHEY_SIMPLEX 
    # org 
    org = (50, 50) 
    
    # fontScale 
    fontScale = 1
       
    # Blue color in BGR 
    color = (255, 0, 0) 
         
    # Line thickness of 2 px 
    thickness = 2
            
    # Using cv2.putText() method 
    return cv2.putText(image, text, org, font,  fontScale, color, thickness, cv2.LINE_AA) 


def get_frame(cap, background_scaled):

    _, frame = cap.read()

    # fetch the mask with retries (the app needs to warmup and we're lazy)

    # e v e n t u a l l y c o n s i s t e n t

    mask = frame # None

    """while mask is None:

        try:

            mask = get_mask(frame)

        except requests.RequestException:

            print("mask request failed, retrying")
    """
    # post-process mask and frame

    mask = post_process_mask(mask)

    frame = hologram_effect(frame)

    # composite the foreground and background

    inv_mask = 1-mask

    for c in range(frame.shape[2]):

        frame[:,:,c] = frame[:,:,c]*mask + background_scaled[:,:,c]*inv_mask

    return frame


# setup access to the *real* webcam

cap = cv2.VideoCapture('/dev/video0')

height, width = 720, 1280

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

cap.set(cv2.CAP_PROP_FPS, 60)


# setup the fake camera

fake = pyfakewebcam.FakeWebcam('/dev/video1', width, height)


# load the virtual background

#background = cv2.imread("/data/background.jpg")

#background_scaled = cv2.resize(background, (width, height))


# frames forever

while True:
    _, frame = cap.read()
    text=hrmd.getText()
    frame = write_text(frame, text)
    #frame = get_frame(cap, background_scaled)

    # fake webcam expects RGB

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    fake.schedule_frame(frame)
