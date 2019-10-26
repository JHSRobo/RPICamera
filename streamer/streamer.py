# Code by Andrew Grindstaff
# Source code adapted from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

# Streams the pi camera stream to port 80 on the raspberry pi.
# Camera module must be enabled through `sudo raspi-config`

import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server
import json

# Default camera settings if for some reason one is not there or the 'reset' button is pressed
default_settings = {'FPS': '60', 'rotation': '0', 'resolution': '640x480'}


def write(dictionary: dict):
    with open("config.json", mode='w') as file:
        file.truncate()
        json.dump(dictionary, file, indent=4)


def write_defaults():
    with open("config.json", mode='w') as settings:
        json.dump(default_settings, settings)


def read():
    with open('config.json', mode='r') as file:
        return json.load(file)


PAGE = """\
<html>
    <body>
        <center>
            <img src="stream.mjpg" width="100%" height="auto"><br><br>
            <form method="post">
                Rotation<br><input type="text" name="rotation"> <br> <br>
                FPS<br><input type="text" name="FPS"><br><br>
                Resolution<br><input type="text" name="resolution"> <br>
                <input type="submit"> <br>
            </form>
            <br>
            <button onclick="window.location.href = 'reset';">Reset Camera</button>
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
        elif self.path == '/reset':
            write_defaults()
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
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
                main()
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        except Exception as e:
            self.send_error(404, 'Error: {}'.format(e))

        self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


try:
    data = read()
except FileNotFoundError:
    write_defaults()
else:
    if len(data) != len(default_settings):
        write_defaults()

output = StreamingOutput()


def main():
    with picamera.PiCamera(resolution=data['resolution'], framerate=data['FPS']) as camera:
        camera.rotation = data['rotation']
        camera.start_recording(output, format='mjpeg')
        try:
            port = 80
            address = ('', port)
            print("Streaming on port {}".format(port))
            streamer = StreamingServer(address, StreamingHandler)
            streamer.serve_forever()
        finally:
            camera.stop_recording()


if __name__ == '__main__':
    main()
