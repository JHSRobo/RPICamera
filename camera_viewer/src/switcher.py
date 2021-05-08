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

# NEED TO ADD SENSOR DATA
# Overlay and timer stack
# Task specific visuals - overlay


class SwitchCameras:
    """The class for handling all the camera logic. Switches and reads the camera, adding an overlay to it"""
    def __init__(self):
        self.verified = {}
        self.num = 0
        self.ip = ""
        self.change = False
        self.cap = None

        try:
            self.config = json.load(open("config.json"))
        except IOError:
            rospy.logerr("Please make config.json if you want to save camera settings")
            self.config = {}

        self.camera_sub = rospy.Subscriber('/rov/camera_select', UInt8, self.change_camera_callback)

        self.camera_thread = threading.Thread(target=self.find_cameras)
        self.camera_thread.setDaemon(True)

        self.camera_thread.start()

    def read(self):
        """Reads a frame from the cv2 video capture and adds the overlay to it"""
        if self.change:
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))
            self.change = False
        ret, frame = self.cap.read()
        if frame is None:
            self.change = True
            return False
        if ret is None:
            return False

        cv2.putText(frame, str(self.num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return frame

    def wait(self):
        """Waits for a camera IP to be put into verified"""
        while not self.verified:
            rospy.logwarn('Waiting for cameras - None connected')
            if rospy.is_shutdown():
                return
            time.sleep(1)

        self.ip = self.verified[1]
        self.num = 1
        rospy.loginfo("Loading capture from camera {}".format(self.num))
        self.cap = cv2.VideoCapture('http://{}:5000'.format(self.ip))

    def change_camera_callback(self, camera_num):
        """ROSPY subscriber to change cameras"""
        if self.num != camera_num.data:
            try:
                num = [x for x in self.verified if self.verified[x] == camera_num.data][0]
            except IndexError:
                pass
            else:
                self.change = True
                self.ip = self.verified[num]
                self.num = num
            rospy.logerr("Changing to camera {}".format(self.num))

    def find_cameras(self):
        """Creates a web server on port 12345 and waits until it gets pinged"""
        app = flask.Flask(__name__)

        @app.route('/', methods=["POST"])
        def page():
            if flask.request.remote_addr not in self.verified:
                try:
                    self.verified[self.give_num(flask.request.remove_addr)] = flask.request.remote_addr
                except IndexError:
                    rospy.logerr('Camera detected, but there\'s no number to assign it to')
                else:
                    rospy.logwarn('Camera at {}, added under {}'.format(flask.request.remote_addr,self.verified[flask.request.remote_addr]))
            return ""

        rospy.logwarn('Camera web server online')
        app.run(host='0.0.0.0', port=12345)

    def give_num(self, ip):
        """Gives the lowest available number to the ip"""
        if ip in self.config:
            return self.config[ip]
        else:
            taken = [self.verified[x] for x in self.verified]
            available = [x for x in range(1, 8) if x not in taken][0]
            return available

    def cleaup(self):
        """Closes the camera thread and attempts to cleanup the program"""
        flask.request.environ.get('werkzeug.server.shutdown')()
        self.camera_thread.terminate()
        self.camera_thread.join()


def main():
    rospy.init_node('pilot_page')
    switcher = SwitchCameras()
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
