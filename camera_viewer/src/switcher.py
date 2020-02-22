#!/usr/bin/env python
# Switcher.py
# By Andrew Grindstaff
# ROS Package to switch between cameras streaming to port 5000
# Streamer code in streamer directory at streamer.py

import cv2
from json import load
from time import sleep
from datetime import datetime
import numpy as np
from threading import Thread
from flask import request, Flask
import rospy
from std_msgs.msg import UInt8

# NEED TO ADD SENSOR DATA
# Overlay and timer stack
# Task specific visuals - overlay


class SwitchCameras:
    """The class for handling all the camera logic. Switches and reads the camera, adding an overlay to it"""
    def __init__(self):
        try:
            self.configed = load(open("config.json"))
        except IOError:
            print "Please make config.json if you want to save camera settings"
            self.configed = {}

        self.verified, self.failed = {}, {}
        self.num = self.cap = None
        self.change = False

    def read(self):
        """Reads a frame from the cv2 video capture and adds the overlay to it"""
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.num))
            self.change = False
        ret, frame = self.cap.read()
        if ret is None:
            return False
        cv2.putText(frame, self.num, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,(255, 255, 255), 2, cv2.LINE_AA)
        return frame

    def screenshot(self):
        """Takes a screenshot and saves it to the ~/Screenshots/CurrentTime.jpg"""
        frame = self.read()
        counter = 0
        while not frame and counter < 5:
            counter += 1
            frame = self.read()

        return cv2.imwrite("~/Screenshots/{}.jpg".format(datetime.now().strftime('%d/%m/%Y-%H:%M:%S')), frame)

    def wait(self):
        """Waits for a camera IP to be put into verified and then assigns numbers"""
        while not self.verified:
            if rospy.is_shutdown():
                return
            sleep(1)

        self.num = self.verified.keys()[0]
        print "Loading capture"
        self.cap = cv2.VideoCapture('http://{}:5000'.format(self.num))

    def change_camera(self, camera_num):
        """rospy subscriber to change cameras"""
        try:
            print self.verified
            num = [x for x in self.verified if self.verified[x]['num'] == camera_num.data][0]
            self.change = True
            self.num = num
        except IndexError:
            pass

    def find_cameras(self):
        """Waits for a request on port 5000"""

        app = Flask(__name__)

        @app.route('/')
        def page():
            if request.remote_addr not in self.verified:
                self.verified[request.remote_addr] = {}
                try:
                    self.give_num(request.remote_addr)
                except IndexError:
                    print 'Camera detected, but all slots are filled'
            return "go away", 200

        print 'Web server online'
        app.run(host='0.0.0.0', port=12345)

    def give_num(self, num):
        if num in self.configed:
            self.verified[num] = self.configed[num]

        taken = [self.verified[x]['num'] for x in self.verified[num]]
        available = [x for x in range(1, 8) if x not in taken]
        try:
            self.verified[num]['num'] = available[0]
            print 'Camera at {}, added under {}'.format(num, self.verified[num]['num'])
        except IndexError:
            print "Camera detected, but all slots filled"

    def record(self, req):

        if req.bool:
            recording = True
        else:
            recording = False

def show_all(cameras):
    """Don't use this - incomplete"""

    def blank_frame(shape):
        """Returns a blank frame for displaying an odd number of video streams"""
        return np.zeros(shape=shape.shape, dtype=np.uint8)

    cameras = list(cameras.values())
    frame = []
    for camera in range(0, len(cameras) - 1, 1):
        ret, frame1 = cv2.VideoCapture('http://{}:5000'.format(cameras[camera])).read()
        if not ret:
            camera -= 1
            continue
        try:
            ret, frame2 = cv2.VideoCapture('http://{}:5000'.format(cameras[camera + 1])).read()
        except IndexError:
            frame2 = blank_frame(frame1)
        else:
            frame2 = blank_frame(frame1) if not ret else frame2
        frame.append(cv2.hconcat([frame1, frame2]))

    for camera in range(1, len(frame)):
        frame[0] = cv2.vconcat([frame[0], frame[camera]])

    cv2.putText(frame[0], 'All', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    return frame[0]


def main():
    switcher = SwitchCameras()

    # ROS Setup
    rospy.init_node('pilot_page')
    rospy.Subscriber('/rov/camera_select', UInt8, switcher.change_camera)

    #rospy.Service('screenshot', camer_viewer.srv.screenshot, switcher.screenshot)
    #rospy.Service('toggle_recording', camera_viewer.srv.toggle_recording, switcher.record)

    switcher_thread = Thread(target=switcher.find_cameras)
    switcher_thread.setDaemon(True)
    switcher_thread.start()

    print 'Waiting for cameras'
    switcher.wait()

    cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    while not rospy.is_shutdown():
        frame = switcher.read()
        if frame is not False:
            cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
