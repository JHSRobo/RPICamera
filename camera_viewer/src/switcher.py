#!/usr/bin/env python
# Switcher.py
# By Andrew Grindstaff
# ROS Package to switch between cameras streaming to port 5000
# Streamer code in streamer directory at streamer.py

import cv2
import json
import signal
import time
import numpy as np
import threading
import os
import rospy
from std_msgs.msg import UInt8

# NEED TO ADD SENSOR DATA
# Overlay and timer stack
# Task specific visuals - overlay


def load_settings():
    verified, failed = {}, {}
    try:
        data = json.load(open("config.json"))
        for index in data:
            if verify(ip_address=data[index]):
                verified[index] = data[index]
            else:
                failed[index] = data[index]
                print 'Camera at {} failed, will try again'.format(data[index])
    except (IOError, KeyError):
        raise NoConfigError
    return verified, failed


class NoConfigError(Exception):
    pass


class NoCamerasError(Exception):
    pass


class SwitchCameras:
    """The class for handling all the camera logic. Switches and reads the camera, adding an overlay to it"""
    def __init__(self):
        try:
            self.verified, self.failed = load_settings()
        except NoConfigError:
            print "Please make config.json if you want to save camera settings"

        self.wait()
        self.num = self.verified.keys()[0]
        self.cap = cv2.VideoCapture('http://{}:5000'.format(self.verified[self.num]))

    def read(self):
        """Reads a frame from the cv2 video capture and adds the overlay to it"""
        ret, frame = self.cap.read()
        if not ret:
            self.camera_failed()
        else:
            cv2.putText(frame, str(self.num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            return frame

    def wait(self):
        """Waits for a camera IP to be put into verified"""
        while not self.verified:
            self.find_cameras()

    def change_camera(self, camera_num):
        """rospy subscriber to change cameras"""
        self.num = camera_num.data
        self.cap.release()
        self.cap = cv2.VideoCapture('http://{}:5000'.format(self.verified[self.num]))

    def find_cameras(self, e=None):
        """
        Finds any cameras on the current networks.
        If e is none, it runs once otherwise e is meant to be a threading.Event
        """
        if e is not None:
            if not isinstance(e, threading.Event):
                raise TypeError("e must be a threading.Event")
            while not e.isSet() and not rospy.is_shutdown():
                self.find_cameras()
                time.sleep(5)
        else:
            available = [x for x in range(1, 8) if str(x) not in self.verified]
            for i in range(2, 255):
                if i == 100 or '192.168.1.{}'.format(i) in self.verified.values():
                    continue
                if verify('192.168.1.{}'.format(i)):
                    try:
                        self.verified[available[0]] = '192.168.1.{}'.format(i)
                        print 'Camera detected at 192.168.1.{}, added under {}'.format(i, j)
                    except IndexError:
                        print 'Cameras detected, but all slots are filled'

    def which_camera(self):
        """rospy service - not being used"""
        def ip():
            return self.verified[self.num]
        rospy.init_node("camera_ip_server")
        s = rospy.Service('current_ip', camera_viewer.srv.current_ip, ip)
        rospy.spin()

    def camera_failed(self):
        print "Camera at {} has failed, switching to a different camera".format(self.verified[self.num])
        try:
            self.failed[self.num] = self.verified[self.num]
            self.verified.pop(self.num, None)
            self.change_camera(self.verified.keys()[0])
        except IndexError:
            raise NoCamerasError


class GracefulKiller:
    kill_now = False

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.kill_now = True


def verify(ip_address):
    """Verifies if an IP address is streaming to port 80"""
    return os.system("ping -c 1 " + ip_address) == 0


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
    graceful_killer = GracefulKiller()
    switcher = SwitchCameras()

    # ROS Setup
    rospy.init_node('pilot_page')
    rospy.Subscriber('/rov/camera_select', UInt8, switcher.change_camera)

    e = threading.Event()
    camera_thread = threading.Thread(target=switcher.find_cameras, args=(e,))
    camera_thread.start()

    #service_thread = threading.Thread(target=switcher.which_camera)
    #service_thread.start()

    switcher.wait()

    cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    while not graceful_killer.kill_now and not rospy.is_shutdown():
        frame = switcher.read()
        if frame is not False:
            cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)
    e.set()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
