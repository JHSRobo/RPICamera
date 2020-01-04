# Code by Andrew Grindstaff
# Source code adapted from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

# https://github.com/waveform80/pistreaming
# https://picamera.readthedocs.io/en/latest/api_camera.html

# Streams the pi camera stream to port 80 on the raspberry pi.
# Camera module must be enabled through `sudo raspi-config`

import io
import picamera
import os
import logging
import socketserver
from threading import Condition
from http import server
import json

# Default camera settings if for some reason one is not there or the 'reset' button is pressed
default_settings = {'FPS': 60, 'rotation': 0, 'resolution': '640x480',
                    'awb_mode': 'cloudy', 'exposure_mode': 'fixedfps',
                    'format': 'mjpeg'}


def kill():
    global streamer
    streamer.shutdown()


def write(dictionary: dict):
    kill()
    with open("config.json", mode='w') as file:
        file.truncate()
        json.dump(dictionary, file, indent=4)
    return dictionary


def write_defaults():
    return write(default_settings)


def read():
    with open('config.json', mode='r') as file:
        return json.load(file)


PAGE = """\
<html>
    <body>
        <center>
            <img src="stream.mjpg" width="auto" height="100%"><br><br>
            <form method="post">
                Rotation<br><input type="text" name="rotation"> <br> <br>
                FPS<br><input type="text" name="FPS"><br><br>
                Resolution<br><input type="text" name="resolution"> <br>
                <input type="submit"> <br>
            </form>
            <br>
            <button onclick="window.location.href = 'reset.html';">Reset Settings To Default</button>
            <br>
            <button onclick="window.location.href = 'restart.html';">Restart Camera, Pulling from Github</button>
            <button onclick="window.location.href = 'shutdown.html';">Kill Camera (NOTE: REQUIRES MANUAL RESTART)</button>
        </center>
    </body>
</html>
"""


class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        elif self.path == '/reset.html':
            write_defaults()
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/restart.html':
            os.system("sudo restart now")
        elif self.path == 'shutdown.html':
            kill()
        else:
            self.send_error(404)
            self.end_headers()

    def do_POST(self):
        try:
            content_length = int(self.headers['Content-Length'])
            data_input = bytes.decode(self.rfile.read(content_length))
            data_input = data_input.split("&")
            current_settings = read()
            change = False
            for val in data_input:
                val = val.split('=')
                if not val[1]:
                    continue
                if val[0] == 'FPS':
                    try:
                        if int(val[1]) > 120 or int(val[1]) < 10:
                            # Please use a number between 10 and 120
                            continue
                    except ValueError:
                        continue
                elif val[0] == 'rotation':
                    try:
                        if int(val[1]) > 360:
                            val[1] = str(int(val[1]) % 360)
                    except ValueError:
                        pass
                if current_settings[val[0]] != val[1]:
                    change = True
                    current_settings[val[0]] = val[1]
            if change:
                write(current_settings)
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        except Exception as e:
            self.send_error(404, 'Error: {}'.format(e))

        self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    global restart, streamer
    try:
        data = read()
    except FileNotFoundError:
        data = write_defaults()
    else:
        if data.keys() != default_settings.keys():
            data = write_defaults()
    #resolution=data['resolution'],
    with picamera.PiCamera(framerate=int(data['FPS'])) as camera:
        camera.rotation = int(data['rotation'])
        camera.awb_mode = data['awb_mode']
        camera.exposure_mode = data['exposure_mode']
        camera.image_effect = 'none'
        camera.start_recording(output, format=data['format'])
        try:
            print("Starting stream")
            streamer.serve_forever()
        except KeyboardInterrupt:
            return
        except PermissionError:
            print("Needs sudo")
            return
        else:
            restart = True
        finally:
            camera.stop_recording()


if __name__ == '__main__':
    restart = False
    output = StreamingOutput()
    port = 80
    address = ('', port)
    print("Streaming on port {}".format(port))
    streamer = StreamingServer(address, StreamingHandler)
    main()
    while restart:
        restart = False
        main()
