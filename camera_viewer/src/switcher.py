# Switcher.py
# By Andrew Grindstaff
# ROS Package to switch between cameras streaming to port 80/stream.mjpg
# Streamer code in streamer directory at streamer.py

import cv2
import json
import requests
import numpy as np
import rospy
from std_msgs.msg import Uint8

# NEED TO ADD ROS ERRORS
# NEED TO ADD SENSOR DATA
# NEED TO ADD CONFIGURATION
# Overlay and timer stack
# Task specific visuals - overlay


def send_msg(msg: str) -> bool:
    """Method to send messages. Right now just prints"""
    print(msg)
    return True


def scan(failed_dict: dict, verified_dict: dict) -> None:
    """Scans all of the addresses in failed_dict to see if any camera came back online and adds them to verified dict"""
    for index in failed_dict:
        try:
            # Check if a camera is online and add it if so
            r = requests.get(f'http://{failed_dict[index]}:80/index.html', timeout=0.025)
        except requests.Timeout or requests.ConnectionError:
            continue
        else:
            if r.status_code == 200:
                if index not in verified_dict:
                    verified_dict[index] = failed_dict[index]
                    send_msg(f'Camera at {failed_dict[index]} is online, added under {index}')
                    failed_dict.pop(index, None)
                else:
                    for j in range(1, 8):
                        if str(j) not in verified_dict:
                            verified_dict[str(j)] = failed_dict[index]
                            send_msg(f'Camera at {failed_dict[index]} is online, added under {j}')
                            failed_dict.pop(index, None)
                            break
                    else:
                        send_msg(f'Camera online at {failed_dict[index]} all cameras are full')


def blank_frame(frame1) -> np.ndarray:
    """Returns a blank frame for displaying an odd number of video streams"""
    return np.zeros(shape=frame1.shape, dtype=np.uint8)


def verify(ip_address: str) -> bool:
    """Verifies if an IP address is streaming to port 80"""
    try:
        r = requests.get(f'http://{ip_address}:80/index.html', timeout=0.05)
    except requests.ConnectTimeout:
        return False
    else:
        if r.status_code == 200:
            return True
        else:
            return False


def show_all(cameras: dict) -> None:
    cameras = list(cameras.values())
    frame = []
    for camera in range(0, len(cameras) - 1, 1):
        ret, frame1 = cv2.VideoCapture(f'http://{cameras[camera]}:80/stream.mjpg').read()
        if not ret:
            camera -= 1
            continue
        try:
            ret, frame2 = cv2.VideoCapture(f'http://{cameras[camera + 1]}:80/stream.mjpg').read()
        except IndexError:
            frame2 = blank_frame(frame1)
        else:
            frame2 = blank_frame(frame1) if not ret else frame2
        frame.append(cv2.hconcat([frame1, frame2]))

    for camera in range(1, len(frame)):
        frame[0] = cv2.vconcat([frame[0], frame[camera]])

    cv2.putText(frame[0], 'All', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    return frame[0]


def find_cameras(ip_addresses: dict) -> None:
    """Finds any cameras on the current networks"""
    verified_address = []
    current_address = ip_addresses.values()
    for i in range(2, 255):
        if f'192.168.1.{i}' in current_address:
            continue
        try:
            r = requests.get(f'http://192.168.1.{i}:80/index.html', timeout=0.05)
        except requests.ConnectionError or requests.ReadTimeout:
            continue
        else:
            if r.status_code == 200:
                verified_address.append(f'192.168.1.{i}')

    available = []
    for j in range(1, 8):
        if str(j) not in ip_addresses:
            available.append(j)

    if available:
        for ip in verified_address:
            send_msg(f'Camera detected at {ip}, added under {available[0]}')
            try:
                ip_addresses[available.pop(0)] = ip
            except IndexError:
                break
    else:
        send_msg("Cameras detected, but all slots filled")


def main():
    with open("config.json") as config:
        data = json.load(config)

    # can't put on one line because they reference each other
    verified = {}
    failed = {}
    for index in data['ip_addresses']:
        if verify(ip_address=data['ip_addresses'][index]):
            verified[index] = data['ip_addresses'][index]
        else:
            failed[index] = data['ip_addresses'][index]
    if not verified:
        # ROS Error
        send_msg("No cameras loaded, quitting")
    find_cameras(verified)

    [send_msg(f'WARNING: Camera at {failed[value]} failed, will try again') for value in failed]

    try:
        num = list(verified.keys())[0]
    except IndexError:
        send_msg("No cameras loaded, quitting")
        return

    rospy.init_node('pilot_page')

    def change_camera(camera_num):
        global cap, num
        cap.release()
        num = camera_num.data
        cap = cv2.VideoCapture(f'http://{verified[str(num)]}:80/stream.mjpg')
    rospy.Subscriber("/rov/camera_select", Uint8, change_camera)

    while True:
        ret, frame = cap.read()
        if not ret:
            send_msg(f"Camera at {verified[str(num)]} has failed, please switch to a different camera")
            failed[str(num)] = verified[str(num)]
            verified.pop(str(num), None)
            try:
                num = list(verified.keys())[0]
                cap.release()
                cap = cv2.VideoCapture(f'http://{verified[num]}:80/stream.mjpg')
            except IndexError:
                send_msg("All cameras have failed")
                return
        else:
            try:
                cv2.putText(frame, f"{data['description'][f'{num}']}: {str(num)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),
                            2, cv2.LINE_AA)
            except KeyError:
                cv2.putText(frame, str(num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow('Camera Feed', frame)

        cv2.waitKey(1)


if __name__ == '__main__':
    main()
