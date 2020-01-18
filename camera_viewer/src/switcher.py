#!/usr/bin/env python
# Switcher.py
# By Andrew Grindstaff
# ROS Package to switch between cameras streaming to port 80/stream.mjpg
# Streamer code in streamer directory at streamer.py

import cv2
import json
import requests
import signal
import time
import numpy as np
import threading
import rospy
from std_msgs.msg import UInt8

# NEED TO ADD SENSOR DATA
# NEED TO ADD CONFIGURATION
# Overlay and timer stack
# Task specific visuals - overlay


def read(cap, num):
    """Purely for ease of use so the function to actually display a frame does not get lost when we inevitably want to
    add more stuff to the frame"""
    ret, frame = cap.read()
    if not ret:
        return False
    else:
        # cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        # cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.putText(frame, str(num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return frame


class SwitchCameras:
    def __init__(self):#, killer):
        # can't put on one line because then they reference each other
        #self.killer = killer
        self.verified = {}
        self.failed = {}
        try:
            data = json.load(open("config.json"))
            for index in data['ip_addresses']:
                if verify(ip_address=data['ip_addresses'][index]):
                    self.verified[index] = data['ip_addresses'][index]
                else:
                    self.failed[index] = data['ip_addresses'][index]
            for value in self.failed:
                print 'Camera at {} failed, will try again'.format(self.failed[value])
        except (IOError, KeyError):
            print "Please make config.json if you want to save settings"

        self.find_cameras()
        if not self.verified:
            print "No cameras available, quitting"
            self.wait_for_cameras()
        self.num = list(self.verified.keys())[0]
        self.cap = cv2.VideoCapture('http://{}:80/stream.mjpg'.format(self.verified[self.num]))

    def wait_for_cameras(self):
        while not self.verified:
            self.find_cameras()

    def find_cameras_search(self):
        while True:#not self.killer.kill_now:
            self.find_cameras()
            time.sleep(5)

    def find_cameras(self):
        """Finds any cameras on the current networks"""
        verified_address = []
        current_address = self.verified.values()
        for i in range(2, 255):
            if '192.168.1.{}'.format(i) in current_address:
                continue
            if verify('192.168.1.{}'.format(i)):
                verified_address.append('192.168.1.{}'.format(i))

        available = []
        for j in range(1, 8):
            if str(j) not in self.verified:
                available.append(j)

        if available:
            for ip in verified_address:
                print('Camera detected at {}, added under {}'.format(ip, available[0]))
                try:
                    self.verified[available.pop(0)] = ip
                except IndexError:
                    break
        else:
            print("Cameras detected, but all slots filled")

    def change_camera(self, camera_num):
        self.num = camera_num.data
        self.cap = cv2.VideoCapture('http://{}:80/stream.mjpg'.format(self.verified[self.num]))

    def which_camera(self):
        def ip():
            return self.verified[self.num]
        rospy.init_node("camera_ip_server")
        s = rospy.Service('current_ip', camera_viewer.srv.current_ip, ip)
        rospy.spin()

    def read(self):
        frame = read(self.cap, self.num)
        if not frame:
            self.camera_failed()
        else:
            return frame

    def camera_failed(self):
        print("Camera at {} has failed, please switch to a different camera".format(self.verified[self.num]))
        self.failed[self.num] = self.verified[self.num]
        self.verified.pop(self.num, None)
        try:
            self.num = list(self.verified)[0]
            self.cap.release()
            self.cap = cv2.VideoCapture('http://{}:80/stream.mjpg'.format(self.verified[self.num]))
        except IndexError:
            print("All cameras have failed")


class GracefulKiller:
    kill_now = False

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.kill_now = True


def blank_frame(frame1):
    """Returns a blank frame for displaying an odd number of video streams"""
    return np.zeros(shape=frame1.shape, dtype=np.uint8)


def verify(ip_address):
    """Verifies if an IP address is streaming to port 80"""
    try:
        r = requests.get('http://{}:80/index.html'.format(ip_address), timeout=0.05)
    except requests.exceptions.RequestException:
        return False
    else:
        return True if r.status_code == 200 else False


def show_all(cameras):
    """Don't use this - incomplete"""
    cameras = list(cameras.values())
    frame = []
    for camera in range(0, len(cameras) - 1, 1):
        ret, frame1 = cv2.VideoCapture('http://{}:80/stream.mjpg'.format(cameras[camera])).read()
        if not ret:
            camera -= 1
            continue
        try:
            ret, frame2 = cv2.VideoCapture('http://{}:80/stream.mjpg'.format(cameras[camera + 1])).read()
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

    # graceful_killer = GracefulKiller()
    switcher = SwitchCameras()
    # ROS Setup
    rospy.init_node('pilot_page')
    rospy.Subscriber('/rov/camera_select', UInt8, switcher.change_camera)

    streaming_thread = threading.Thread(target=stream, args=(switcher,))#, args=(graceful_killer,))
    streaming_thread.start()
    streaming_thread.run()

    camera_thread = threading.Thread(target=switcher.find_cameras_search)
    camera_thread.start()
    camera_thread.run()

    # serviceThread = threading.Thread(target=switcher.which_camera)
    # serviceThread.start()


def stream(switcher):#, killer):
    while not False:#rospy.is_shutdown():
        print 123
        frame = switcher.read()
        if frame is not False:
            cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()





