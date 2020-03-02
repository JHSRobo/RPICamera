#!/usr/bin/env python
# Switcher.py
# By Andrew Grindstaff
# ROS Package to switch between cameras streaming to port 5000
# Streamer code in streamer directory at streamer.py

import cv2
import json
import time
import numpy as np
import threading
import flask
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# NEED TO ADD SENSOR DATA
# Overlay and timer stack
# Task specific visuals - overlay


class SwitchCameras:
    """The class for handling all the camera logic. Switches and reads the camera, adding an overlay to it"""
    def __init__(self):
        try:
            self.config = json.load(open("config.json"))
        except IOError:
            print "Please make config.json if you want to save camera settings"
            self.config = {}

        self.camera_sub = rospy.Subscriber('/rov/camera_select', UInt8, self.change_camera_callback)
        self.pub = rospy.Publisher('/rov/camera_stream', Image, queue_size=1)
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()

        self.verified, self.failed = {}, {}
        self.num, self.cap = None, None
        self.change, self.frame = False, False

    def read(self):
        """Reads a frame from the cv2 video capture and adds the overlay to it"""
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.num))
            self.change = False
        ret, frame = self.cap.read()
        if frame is None:
            self.change = True
        if ret is None or frame is None:
            return False
        self.frame = frame
        cv2.putText(frame, self.num, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return frame

    def wait(self):
        """Waits for a camera IP to be put into verified"""
        while not self.verified:
            if rospy.is_shutdown():
                return
            time.sleep(1)

        self.num = self.verified.keys()[0]
        print "Loading capture"
        self.cap = cv2.VideoCapture('http://{}:5000'.format(self.num))

    def change_camera_callback(self, camera_num):
        """rospy subscriber to change cameras"""
        try:
            if self.num == camera_num.data:
                return
            num = [x for x in self.verified if self.verified[x]['num'] == camera_num.data][0]
            self.change = True
            self.num = num
        except IndexError:
            pass

    def find_cameras(self):
        """Creates a web server on port 12345 and waits until it gets pinged"""
        app = flask.Flask(__name__)

        @app.route('/', methods=["POST"])
        def page():
            if flask.request.remote_addr not in self.verified:
                self.verified[flask.request.remote_addr] = {}
                try:
                    self.give_nums()
                except IndexError:
                    print 'Camera detected, but all slots are filled'
            return "", 200

        print 'Web server online'
        app.run(host='0.0.0.0', port=12345)

    def give_nums(self):
        for x in self.config:
            if x in self.verified:
                self.verified[x]['num'] = self.config[x]['num']
                print 'Camera at {}, added under {}'.format(x, self.config[x]['num'])
            else:
                self.failed[x] = self.verified[x]

        taken = [self.verified[x]['num'] for x in self.verified if 'num' in self.verified[x]]
        available = [x for x in range(1, 8) if x not in taken]
        for x in self.verified:
            if 'num' not in self.verified[x]:
                self.verified[x]['num'] = available.pop(0)
                print 'Camera at {}, added under {}'.format(x, self.verified[x]['num'])

    def relay(self, pub, rate):
        time.sleep(2)
        while not rospy.is_shutdown():
            pub.publish(self.bridge.cv2_to_img_msg(self.frame))
            rate.sleep()


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

    camera_thread = threading.Thread(target=switcher.find_cameras)
    camera_thread.setDaemon(True)
    camera_thread.start()

    relay_thread = threading.Thread(target=switcher.relay)
    relay_thread.setDaemon(True)

    print 'Waiting for cameras'
    switcher.wait()
    relay_thread.start()

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
