#!/usr/bin/env python
"""
foo bar
"""
import json
import subprocess  # For starting rosbag.
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from os import curdir, sep
import rospy  # ROS
from src.user_input_ros import UserFormROS
import interaction_launcher

PORT = 8080


class RRHandler(BaseHTTPRequestHandler):
    """
    A Python HTTP server for the RR User Input web interface.
    """

    def __init__(self, ros_node, shared, *args, **kwargs):
        """
        Use init to get the reference to ROS.
        """
        self.ros_node = ros_node
        self.shared = shared
        BaseHTTPRequestHandler.__init__(self, *args, **kwargs)

    def do_GET(self):
        """
        Handle all GET requests. Only the main file is ever needed.
        """
        if self.path == "/":
            try:
                fh = open(curdir + sep + "index.html")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(fh.read())
                fh.close()
            except IOError:
                self.send_error(404, "File not found: index.html")

        elif self.path == "/status":
            if "t" in shared:
                if not shared["t"].poll() is None:
                    del shared["t"]

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"running": "t" in shared, "msg": shared.get("msg")}))

        else:
            self.send_response(404)
            self.end_headers()

        return

    def do_POST(self):
        """
        Handle all commands from the UI. The POST body is a JSON object like:
        { type: "INTERACTION", cmd: "START" }
        """
        if self.path == "/command":
            # Read and parse the body.
            body = self.rfile.read(int(self.headers.getheader("Content-Length", 0)))
            msg = json.loads(body)

            # TEGA means it's actually an animation command.
            if msg["type"] == "TEGA":
                self.ros_node.send_tega_animation(self.add_quotes(msg["cmd"]))
            else:
                self.ros_node.send_message(
                    self.add_quotes(msg["type"]), self.add_quotes(msg["cmd"]))

            self.send_response(200)
            self.end_headers()
        elif self.path == "/launch":

            # Don't start it again if it's still running.
            if "t" in shared:
                if shared["t"].poll():
                    del shared["t"]
                else:
                    self.send_response(412)
                    self.end_headers()
                    return

            # Read and parse the body.
            body = self.rfile.read(int(self.headers.getheader("Content-Length", 0)))
            msg = json.loads(body)
            print msg
            print interaction_launcher.check_name(msg["experimenter"])
            print interaction_launcher.check_pid(msg["participant"])
            print interaction_launcher.check_session(int(msg["session"]))

            # Validation.
            if False in [interaction_launcher.check_name(msg["experimenter"]),
                    interaction_launcher.check_pid(msg["participant"]),
                    interaction_launcher.check_session(int(msg["session"]))]:
                self.send_response(400)
                self.end_headers()
                return

            print "launching"
            shared["t"] = subprocess.Popen(["./launch_interaction.py",
                    "-e", msg["experimenter"],
                    "-p", msg["participant"],
                    "-s", msg["session"],
                    "-r", msg.get("restart", "intro"),
                    "-n"],
               shell=False)

            # Save the params for viewing later.
            shared["msg"] = msg


            self.send_response(200)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()

        return

    def add_quotes(self, s):
        """
        The ROS message definitions have extra quotes around them, so add extra
        ones here too.
        """
        return '"' + s + '"'


if __name__ == '__main__':
    # Initialize the ROS node.
    ros_node = UserFormROS(rospy.init_node('rr_user_input_form', anonymous=False))
    shared = {}

    # HTTPServer creates a new instance of RRHandler for every request, bit it
    # needs access to a global ROS node handle. This lambda gives it access
    # via closure.
    server = HTTPServer(("", PORT),
            lambda *args, **kwargs: RRHandler(ros_node, shared, *args, **kwargs))
    print "HTTP server running"
    # Because rospy catches signals, we can't use server.serve_forever().
    # Instead, try to handle requests one at a time (with a timeout), checking
    # whether ROS is still running in between.
    server.timeout = 0.5
    while not rospy.is_shutdown():
        server.handle_request()

    print "Shutting down..."
    server.server_close()
