# Code by Andrew Grindstaff
# Source code adapted from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

# https://github.com/waveform80/pistreaming
# https://picamera.readthedocs.io/en/latest/api_camera.html

# Streams the pi camera stream to port 80 on the raspberry pi.
# Camera module must be enabled through `sudo raspi-config`

import io
import picamera
import logging
import sys
import socketserver
from threading import Condition
from http import server
import json

# Default camera settings if for some reason one is not there or the 'reset' button is pressed
default_settings = {'FPS': 60, 'rotation': 0, 'resolution': '640x480',
                    'awb_mode': 'cloudy', 'exposure_mode': 'fixedfps',
                    'format': 'mjpeg'}


def write(dictionary: dict):
    with open("config.json", mode='w') as file:
        file.truncate()
        json.dump(dictionary, file, indent=4)
    Stream.restart()
    return dictionary


def write_defaults():
    return write(default_settings)


def read():
    try:
        with open('config.json', mode='r') as file:
            data = json.load(file)
    except FileNotFoundError:
        with open("config.json", mode='w') as file:
            file.truncate()
            json.dump(default_settings, file, indent=4)
        data = default_settings
    else:
        if data.keys() != default_settings.keys():
            with open("config.json", mode='w') as file:
                file.truncate()
                json.dump(default_settings, file, indent=4)
            data = default_settings
    return data


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
            <br> <br> <br>
            <button onclick="window.location.href = 'shutdown.html';">Kill Switch for manual testing</button>
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
                    with Stream.output.condition:
                        Stream.output.condition.wait()
                        frame = Stream.output.frame
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
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
            write_defaults()
        elif self.path == 'shutdown.html':
            Stream.streamer.shutdown()
            sys.exit()
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
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
            if change:
                write(current_settings)
        except Exception as e:
            self.send_error(404, 'Error: {}'.format(e))

        self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class Stream:
    output = StreamingOutput()
    streamer = StreamingServer(('', 80), StreamingHandler)

    @staticmethod
    def run():
        data = read()
        with picamera.PiCamera(framerate=int(data['FPS'])) as camera:
            camera.rotation = int(data['rotation'])
            camera.awb_mode = data['awb_mode']
            camera.exposure_mode = data['exposure_mode']
            camera.image_effect = 'none'
            camera.start_recording(Stream.output, format=data['format'])
            try:
                print("Starting stream")
                Stream.streamer.serve_forever()
            except KeyboardInterrupt:
                return
            except PermissionError:
                print("Needs sudo")
                return
            finally:
                Stream.streamer.shutdown()
                camera.stop_recording()

    @staticmethod
    def restart():
        Stream.streamer.shutdown()
        Stream.run()


def main():
    Stream.run()


if __name__ == '__main__':
    main()
