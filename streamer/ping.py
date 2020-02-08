import requests
import time

r = requests.post(url="192.168.1.100:80")
while r.status_code != 200:
    time.sleep(1)
    r = requests.post(url="192.168.1.100:80")
