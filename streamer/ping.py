import requests
import time

try:
    r = requests.post(url="http://192.168.1.100:80").status_code
except Exception:
    r = 1
while r != 200:
    time.sleep(1)
    try:
        r = requests.post(url="http://192.168.1.100:80").status_code
    except Exception:
        r = 1
