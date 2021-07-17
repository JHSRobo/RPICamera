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
from std_msgs.msg import UInt8, Float32
from sensor_msgs.msg import Image

# NEED TO ADD SENSOR DATA
# Overlay and timer stack
# Task specific visuals - overlay


class CameraSwitcher:
    """The class for handling all the camera logic. Switches and reads the
    camera, adding an overlay to it.
    """
    def __init__(self):
        # self.verified is a dictionary that has keys of camera numbers and
        # values of ip addresses -
        # self.verified = {
        #  1: "192.168.1.101",
        #  2: "192.168.1.102"
        # }
        self.verified = {}
        # self.num is an integer that represents the current camera number and
        # is the key for self.verified
        self.num = 0
        self.change = False
        self.cap = None

        try:
            self.config = json.load(open("/home/jhsrobo/config.json"))
        except IOError:
            rospy.logwarn("camera_viewer: please make config.json if you want to save camera settings")
            self.config = {}

        self.camera_sub = rospy.Subscriber('/rov/camera_select', UInt8, self.change_camera_callback)

        self.depth_sub = rospy.Subscriber('/depth_sensor', Float32, self.change_depth_callback)
        self.depth = 0

        self.gpio_sub = rospy.Subscriber('/gpio_control', UInt8, self.change_gpio_callback)
        self.electromags = {11: [-1, "Left pad"],
                           13: [-1, "Right pad"],
                           15: [-1, "Quadrat"],
                           18: [-1, "Micro ROV"]
                           }

        self.camera_thread = threading.Thread(target=self.find_cameras)
        self.camera_thread.setDaemon(True)

        self.camera_thread.start()

    @property
    def ip(self):
        """Ensures that the IP of the camera is always the correct number
        without sacraficing redability. Otherwise, returns False
        """
        try:
            return self.verified[self.num]
        except KeyError:
            rospy.logerr("camera_viewer: passed a camera number that doesn't exist")
            if len(self.verified.keys()) == 0:
                return ""
            return self.verified[self.verified.keys()[0]]

    def read(self):
        """Reads a frame from the cv2 video capture and adds the overlay to it"""
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
            self.change = False

        ret, frame = self.cap.read()
        if frame is None:
            rospy.logerr('camera_viewer: camera failed - please wait for it to refresh or switch cameras')
            self.change = True
            return False
        elif ret is None:
            rospy.logwarn('camera_viewer: ret is None, can\'t display new frame')
            return False
        else:
            cv2.putText(frame, str(self.num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, "{:.2f}".format(self.depth), (1260 - cv2.getTextSize("{:.2f}".format(self.depth), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0][0], 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            height = 80
            for x in self.electromags:
                cv2.putText(frame, "{}: {}".format(x[1], "On" if x[0] == 1 else "Off"), ((1260 - cv2.getTextSize("{}: {}".format(x[1], "On" if x[0] == 1 else "Off")), cv2.FONT_HERSHEY_COMPLEX, 1, 2)[0][0], height), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                height += 40
            return frame

    def wait(self):
        """Waits for a camera IP to be put into verified"""
        rospy.loginfo('camera_viewer: waiting for cameras - None connected')
        while not self.verified:
            if rospy.is_shutdown():
                return
            rospy.logdebug('camera_viewer: still no cameras connected')
            time.sleep(1)

        self.num = 1
        rospy.loginfo("camera_viwer: loading capture from camera {}".format(self.num))
        if self.ip:
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
        else:
            # lol something is really wrong
            rospy.logerr("camera_viwer: there is no camera at spot 1 after waiting.")

    def change_camera_callback(self, camera_num):
        """ROSPY subscriber to change cameras"""
        if not len(self.verified.keys()):
          rospy.logerr('camera_viewer: passed a value when none are initialized')
        elif self.num != camera_num.data:
            if self.ip:
                self.change = True
                self.num = camera_num.data
                rospy.loginfo("camera_viewer: changing to camera {}".format(self.num))

    def change_depth_callback(self, depth):
        """ROSPY subscriber to change depth"""
        self.depth = depth.data

    def change_gpio_callback(self, data):
        """ROSPY subscriber to change gpio info"""
        if data.data in self.electromags:
            self.electromags[data.data][0] = self.electromags[data.data][0] * -1

    def find_cameras(self):
        """Creates a web server on port 12345 and waits until it gets pinged"""
        app = flask.Flask(__name__)

        @app.route('/', methods=["POST", "GET"])
        def page():
            rospy.logdebug('camera_viewer: ping from {}'.format(flask.request.remote_addr))
            if flask.request.remote_addr not in self.verified.values() and len(flask.request.form.keys()) > 0:
                if flask.request.form.keys()[0] in self.config.keys():
                    self.verified[self.config[flask.request.form.keys()[0]]] = flask.request.remote_addr
                else:
                    self.verified[self.give_num(flask.request.remote_addr)] = flask.request.remote_addr
                rospy.loginfo('camera_viewer: cameras currently connected: {}'.format(self.verified))
            return ""

        rospy.loginfo('camera_viewer: camera web server online')
        app.run(host='0.0.0.0', port=12345)

    def give_num(self, ip):
        """Gives the lowest available number to the ip"""
        if ip in self.config:
            return self.config[ip]
        else:
            try:
                available = [num for num in range(1, 8) if num not in self.verified and num not in self.config.values()][0]
            except IndexError:
                rospy.logerr('camera_viwer: camera detected, but there are no available numbers')
            return available

    def cleaup(self):
        """Closes the camera thread and attempts to cleanup the program"""
        flask.request.environ.get('werkzeug.server.shutdown')()
        self.camera_thread.terminate()
        self.camera_thread.join()


def main():
    rospy.init_node('pilot_page')
    switcher = CameraSwitcher()
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
