import requests
import time

while True:
    time.sleep(1)
    requests.get(url="http://192.168.1.100:12345")
