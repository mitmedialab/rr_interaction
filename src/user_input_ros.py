"""
Jacqueline Kory Westlund
November 2017

The MIT License (MIT)

Copyright (c) 2017 Personal Robots Group

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rospy  # ROS
import logging  # Log messages.

from rr_msgs.msg import UserInput  # Send user responses.
from std_msgs.msg import Header  # standard ROS msg header


class UserFormROS(object):
    """ ROS functions for sending user input responses. """

    def __init__(self, ros_node):
        """ Initialize ROS. """
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up user input form's ROS node...")

        # We get a reference to the main ros node so we can do callbacks
        # to publish messages, and subscribe to stuff.
        self.ros_node = ros_node

        # We will publish user input messages with results from the form.
        self.user_input_pub = rospy.Publisher('/rr/user_input', UserInput,
                                              queue_size=10)

    def send_message(self, response_type, response):
        """ Publish UserInput message. """
        if self.user_input_pub is not None:
            self._logger.info("Publishing: type {}, response {}".format(
                 response_type, response))
            msg = UserInput()
            # add header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.response_type = response_type
            msg.respones = response
            self.user_input_pub.publish(msg)
            rospy.loginfo(msg)
