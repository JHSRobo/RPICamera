#!/usr/bin/env python
# Switcher.py
# By Andrew Grindstaff
# ROS Package to switch between cameras streaming to port 5000
# Streamer code in streamer directory at streamer.py

import cv2
import json
import time
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
            return False
        if ret is None:
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

        print "Loading capture from camera {}".format(self.num)
        self.cap = cv2.VideoCapture('http://{}:5000'.format(self.num))

    def change_camera_callback(self, camera_num):
        """ROSPY subscriber to change cameras"""
        if self.num != camera_num.data:
            try:
                num = [x for x in self.verified if self.verified[x]['num'] == camera_num.data][0]
            except IndexError:
                pass
            else:
                self.change = True
                self.num = num

    def find_cameras(self):
        """Creates a web server on port 12345 and waits until it gets pinged"""
        app = flask.Flask(__name__)

        @app.route('/', methods=["POST"])
        def page():
            if flask.request.remote_addr not in self.verified:
                self.verified[flask.request.remote_addr] = {}
                try:
                    self.give_num(flask.request.remote_addr)
                except IndexError:
                    print 'Camera detected, but all slots are filled'
            return ""

        print 'Web server online'
        app.run(host='0.0.0.0', port=12345)

    def give_num(self, ip):
        """Gives the lowest available number to the ip"""
        if ip in self.config:
            self.verified[ip]['num'] = self.config[ip]['num']
        else:
            taken = [self.verified[x]['num'] for x in self.verified if 'num' in self.verified[x]]
            available = [x for x in range(1, 8) if x not in taken][0]
            self.verified[ip]['num'] = available
        print 'Camera at {}, added under {}'.format(ip, self.verified[ip]['num'])
        if self.num is False:
            self.num = self.verified[ip]['num']
            print 'Camera number is {}'.format(self.verified[ip]['num'])

    def relay(self, pub, rate):
        """Relay to publish the image -- should work well but IDRK"""
        time.sleep(2)
        while not rospy.is_shutdown():
            pub.publish(self.bridge.cv2_to_img_msg(self.frame))
            rate.sleep()


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
