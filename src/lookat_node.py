#!/usr/bin/env python
"""
Jacqueline Kory Westlund
February 2018

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

import logging  # Log messages.
import random  # For choosing backchannel actions.
import rospy  # ROS
import time  # For sleep.
import threading  # Timer for randomly backchanneling.
import rr_commons as rr_commons  # Common constants, such as lookat vectors.
from rr_msgs.msg import InteractionState  # Send state to the audio entrainer.
from r1d1_msgs.msg import TegaAction  # Send commands to Tega.
from sar_opal_msgs.msg import OpalCommand  # Send commands an Opal device.
from std_msgs.msg import Header  # Standard ROS msg header.


def on_state_msg(data):
    """ When we receive an interaction state message, perform lookat behaviors
    as appropriate.
    """
    LOGGER.debug("Got state message: {}".format(data.state))
    global USER_STORY
    # If it is the user's turn to speak or act, we should mostly look at them.
    # Elsewhere, the robot is told to look at the user when it is the user's
    # turn to speak.

    # But, if this is during a story, we can occasionally glance at the tablet.
    if data.is_participant_turn:
        return

    if ("start child story retell" in data.state or
            "start child story create" in data.state):
        # Look at the tablet and start alternating.
        send_lookat(rr_commons.LOOKAT["TABLET"])
        USER_STORY = True
        do_story_lookat(False)
    elif ("end child story retell" in data.state or
            "end child story create" in data.state):
        # At the end of the user's story, stop glancing at the tablet.
        USER_STORY = False


def on_opal_msg(data):
    """ Use OpalCommand messages to know whether to look at the tablet. """
    LOGGER.debug("Got opal command message: {}".format(data.command))
    # If the tablet page is told to change or something new is loaded on the
    # tablet, look at the tablet for a few seconds, then back at the child.
    if (data.command == OpalCommand.NEXT_PAGE or
            data.command == OpalCommand.PREV_PAGE or
            data.command == OpalCommand.STORY_SELECTION or
            data.command == OpalCommand.STORY_SHOW_BUTTONS or
            data.command == OpalCommand.LOAD_OBJECT or
            data.command == OpalCommand.STORY_GO_TO_PAGE):
        # Look at the tablet.
        send_lookat(rr_commons.LOOKAT["TABLET"])
        # Wait for a few seconds.
        time.sleep(2.5 + random.uniform(-0.5, 0.5))
        # Look back at the child.
        send_lookat(rr_commons.LOOKAT["USER"])


def do_story_lookat(looking_at_user):
    """ During a story, look between the user and the tablet. """
    # If the user is no longer telling a story, stop sending lookats.
    if not USER_STORY:
        return

    # When the child is telling or retelling a story, look at the tablet most
    # of the time, but glance at the child sometimes. Look at the child about
    # every 5s, for about 2.5s. Then look back at the tablet. These intervals
    # are what were used in the SR2 story study during child stories.
    if looking_at_user:
        send_lookat(rr_commons.LOOKAT["TABLET"])
        # Call this function again in a little while to look back at the user.
        threading.Timer(5 + random.uniform(-0.5, 0.5),
                        do_story_lookat(False)).start()
    else:
        send_lookat(rr_commons.LOOKAT["USER"])
        # Call this function again in a little while to look back at the
        # tablet.
        threading.Timer(2.5 + random.uniform(-0.5, 0.5),
                        do_story_lookat(True)).start()


def send_lookat(lookat):
    """ Publish a Tega command message containing motion to do or speech to
    play.
    """
    if not TEGA_PUB:
        LOGGER.warning("No TegaAction ROS publisher is setup!")
        return
    LOGGER.info("Sending Tega lookat command...")
    # Build message.
    msg = TegaAction()
    # Add header.
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    # Add the lookat command.
    if lookat is not None:
        msg.do_look_at = True
        msg.look_at = lookat
    # Send message.
    TEGA_PUB.publish(msg)
    LOGGER.debug(msg)


if __name__ == "__main__":
    """ Send lookat commands to the robot based on the current interaction
    state.
    """
    # Set up logger.
    LOGGER = logging.getLogger(__name__)
    LOGGER.info("Initializing lookat handler...")

    try:
        # Initialize our ROS node.
        rospy.init_node("lookat_handler", anonymous=False)

        LOGGER.info("We will publish to topics: /tega")
        LOGGER.info("We subscribe to topics: /rr/state, /rr_opal_command")

        # We will send action or speech commands to the Tega robot.
        TEGA_PUB = rospy.Publisher('/tega', TegaAction, queue_size=10)

        # Interaction state messages.
        rospy.Subscriber('/rr/state', InteractionState, on_state_msg)

        # Opal Command messages, so we can see when the tablet page changes.
        rospy.Subscriber('/rr/opal_command', OpalCommand, on_opal_msg)

        # Spin.
        rospy.spin()

    # If roscore isn't running or shuts down unexpectedly, we won't continue.
    except rospy.ROSInterruptException:
        print "ROS node shutdown"
