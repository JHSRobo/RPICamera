# Web streaming example
# Source code from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

# streams to port 80 on the raspberry pi

import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server
import json

try:
    with open('config.json', mode='r') as json_file:
        data = json.load(json_file)
except FileNotFoundError:
    with open("config.json", mode='w') as config:
        json.dump({'FPS': '60', 'rotation': '0', 'resoloution': '640x480'}, config)


def writer(filename: str, dictionary: dict):
    with open(filename, mode='w') as config:
        config.truncate()
        json.dump(dictionary, config, indent=4)
        return config


PAGE = """\
<html>
<body>
<center><img src="stream.mjpg" width="100%" height="auto"></center>
<br><br>
<center><form method="post">
Rotation<br><input type="text" name="rotation">
<br><br>
FPS<br><input type="text" name="FPS"> 
<br><br>
Resoloution<br><input type="text" name"resoloution"> 
<br>
<input type="submit"></form>
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
        else:
            self.send_error(404)
            self.end_headers()

    def do_POST(self):
        try:
            content_length = int(self.headers['Content-Length'])
            data_input = bytes.decode(self.rfile.read(content_length))
            data_input = data_input.split("&")
            for val in data_input:
                val = val.split('=')
                if val[0] == 'FPS':
                    try:
                        if int(val[1]) > 120 or int(val[1]) < 10:
                            # Please use a number between 10 and 120
                            continue
                    except ValueError:
                        continue
                elif val[0] == 'resoloution':
                    try:
                        width, height = val[1].split("x")
                    except ValueError:
                        # Please use WIDTHxHEIGHT
                        pass
                elif val[0] == 'rotation':
                    try:
                        if int(val[1]) > 360:
                            val[1] = str(int(val[1]) % 360)
                    except ValueError:
                        pass
                data[val[0]] = val[1]
            writer("config.json", data)
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        except:
            self.send_error(404, 'Bad request submitted.')

        self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


with picamera.PiCamera(resolution=data['resoloution'], framerate=data['FPS']) as camera:
    output = StreamingOutput()
    # Uncomment the next line to change your Pi's Camera rotation (in degrees)
    camera.rotation = data['rotation']
    camera.start_recording(output, format='mjpeg')
    try:
        port = 80
        address = ('', port)
        print(f"Streaming on port {port}")
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    finally:
        camera.stop_recording()
