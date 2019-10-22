import cv2
import json
import requests
import numpy as np

def scan(failed_dict, verified_dict):
    for address in list(failed_dict.keys()):  # Check if a camera is online and add it if so
        try:
            r = requests.get(f'http://{failed_dict[address]}:8000/index.html', timeout=0.025)
        except requests.Timeout or requests.ConnectionError:
            pass
        else:
            if r.status_code == 200:
                if address not in capture_list.keys():
                    capture_list[address] = cv2.VideoCapture(f"http://{failed_dict[address]}:8000/stream.mjpg")
                    verified_dict[address] = failed_dict[address]
                    print(f'Camera at {failed_dict[address]} is online, added under {address}')
                    camera_num_list.append(ord(str(address)))
                    failed_dict.pop(address, None)
                else:
                    for j in range(1, 8):
                        if str(j) not in capture_list.keys():
                            capture_list[str(j)] = cv2.VideoCapture(f"http://{failed_dict[address]}:8000/stream.mjpg")
                            verified_dict[str(j)] = failed_dict[address]
                            print(f'Camera at {failed_dict[address]} is online, added under {j}')
                            camera_num_list.append(ord(str(j)))
                            failed_dict.pop(address, None)
                            break
                    else:
                        print(f'Camera online at {failed_dict[address]} all cameras are full')


def blank_frame(frame1):
    return np.zeros(shape=frame1.shape, dtype=np.uint8)

def show_all(cameras: dict):
    cameras = list(cameras.values())
    frame = []
    for camera in range(0, len(cameras) - 1, 1):
        ret, frame1 = cv2.VideoCapture(f'http://{cameras[camera]}:8000/stream.mjpg').read()
        if not ret:
            camera -= 1
            continue
        try:
            ret, frame2 = cv2.VideoCapture(f'http://{cameras[camera + 1]}:8000/stream.mjpg').read()
        except IndexError:
            frame2 = blank_frame(frame1)
        else:
            frame2 = blank_frame(frame1) if not ret else frame2
        frame.append(cv2.hconcat([frame1, frame2]))

    for camera in range(1, len(frame)):
        frame[0] = cv2.vconcat([frame[0], frame[camera]])

    cv2.putText(frame[0], 'All', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    return frame[0]



def verify(ip_addresses: dict):
    camera_dict = {}
    failed_dict = {}
    for address in ip_addresses.keys():
        try:
            r = requests.get(f'http://{ip_addresses[address]}:8000/index.html', timeout=0.05)
        except requests.ConnectTimeout:
            failed_dict[address] = ip_addresses[address]
        else:
            if r.status_code == 200:
                camera_dict[address] = ip_addresses[address]
            else:
                failed_dict[address] = ip_addresses[address]

    return camera_dict, failed_dict


def ping_all(ip_addresses: dict):
    verified_address = []
    for i in range(2, 255):
        if f'192.168.1.{i}' in ip_addresses.values():
             continue
        try:
            r = requests.get(f'http://192.168.1.{i}:8000/index.html', timeout=0.05)
        except requests.ConnectionError or requests.ReadTimeout:
            pass
        else:
            if r.status_code == 200:
                verified_address.append(f'192.168.1.{i}')

    for ip in verified_address:
        if ip not in list(ip_addresses.values()):
            for j in range(1, 8):
                if str(j) not in ip_addresses.keys():
                    ip_addresses[str(j)] = ip
                    print(f'Camera detected at {ip}, added under {j}')
                    break
    return ip_addresses


def main():
    with open("config.json") as config:
        data = json.load(config)

    verified_dict, failed_dict = verify(data['ip_address'])
    verified_dict = ping_all(verified_dict)

    [print(f'WARNING: Camera at {failed_dict[value]} failed, will try again') for value in failed_dict.keys()]

    camera_num_list = [ord(camera) for camera in verified_dict]  # for detecting key presses
    #camera_num_list.append(ord('9'))

    num = '1'
    try:
        cap = cv2.VideoCapture(f'http://{list(verified_dict.values())[0]}:8000/stream.mjpg')
    except IndexError:
        print("No cameras loaded, quitting")
        return
    while True:
        if num == '9':
            cv2.imshow('Camera Feed', show_all(verified_dict))

        else:
            ret, frame = cap.read()
            if not ret:
                print(f"Camera at {verified_dict[str(num)]} has failed, please switch to a different camera")
                failed_dict[str(num)] = verified_dict[str(num)]
                verified_dict.pop(str(num), None)
                camera_num_list.remove(ord(num))
                try:
                    num = list(verified_dict.keys())[0]
                    cap.release()
                    cap = cv2.VideoCapture(f'http://{verified_dict[num]}:8000/stream.mjpg')
                except IndexError:
                    print("All cameras have failed")
                    return
            else:
                try:
                    cv2.putText(frame, f"{data['description'][f'{num}']}: {str(num)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),
                                2, cv2.LINE_AA)
                except KeyError:
                    cv2.putText(frame, str(num), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow('Camera Feed', frame)

        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            cap.release()
            break

        elif k & 0xFF == ord('9'):
            num = '9'

        elif k & 0xFF in camera_num_list:
            cap.release()
            num = f'{k - 48}'
            cap = cv2.VideoCapture(f'http://{verified_dict[str(num)]}:8000/stream.mjpg')

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
