"""
Jacqueline Kory Westlund
August 2017

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
import datetime  # For header times and timeouts.
import time  # For sleep.
import logging  # For log messages.
from r1d1_msgs.msg import TegaAction  # Send commands to Tega.
from r1d1_msgs.msg import TegaState  # ROS msgs to get info from Tega.
from sar_opal_msgs.msg import OpalCommand  # Send commands an Opal device.
from sar_opal_msgs.msg import OpalAction  # Get Opal device state and actions.
from rr_msgs.msg import EntrainAudio  # Send audio to the audio entrainer.
from rr_msgs.msg import InteractionState  # Send state to the audio entrainer.
from std_msgs.msg import Header  # Standard ROS msg header.
from std_msgs.msg import String  # Get string state from audio entrainer.


class RosNode(object):
    """ ROS node: Set up rostopics we publish, subscribe to rostopics
    we care about, functions to send and receive messages.
    """
    # pylint: disable=too-many-instance-attributes

    def __init__(self, queue):
        """ Initialize ROS """
        # We get a reference to the main node's queue so we can give it
        # messages.
        self._game_node_queue = queue

        # Set up logger
        self._logger = logging.getLogger(__name__)

        # Initialize the flags we use to track responses from the robot and
        # from the user.
        self._robot_doing_action = False
        self._robot_speaking = False
        self._response_received = None
        self._touched_object = ""
        self.start_response_received = False

        # We don't start out waiting for anything.
        self._waiting_for_start = False
        self._waiting_for_robot_speaking = False

        # Set up rostopics we publish:
        self._logger.info("We will publish to topics: rr/opal_command, /tega, "
                          + "/rr/entrain_audio, /rr/state")
        # Commands to the Opal game.
        self._opal_pub = rospy.Publisher('/rr/opal_command', OpalCommand,
                                         queue_size=10)
        # Commands to the Tega robot.
        self._tega_pub = rospy.Publisher('/tega', TegaAction, queue_size=10)
        # Commands to entrain audio.
        self._entrainer_pub = rospy.Publisher('/rr/entrain_audio',
                                              EntrainAudio,
                                              queue_size=10)
        # Interaction state messages.
        self._state_pub = rospy.Publisher('/rr/state', InteractionState,
                                          queue_size=10)

        # Set up rostopics we subscribe to:
        self._logger.info("Subscribing to topics: /rr/opal_action, "
                          + "/tega_state, /rr/audio_entrainer")
        # State from the Opal game.
        rospy.Subscriber('/rr/opal_action', OpalAction,
                         self.on_opal_action_msg)
        # State from the Tega robot.
        rospy.Subscriber('/tega_state', TegaState, self.on_tega_state_msg)
        # State from the audio entrainer.
        rospy.Subscriber('/rr/audio_entrainer', String, self.on_entrainer_msg)

    # TODO check whether there are any new opal commands not included here.
    def send_opal_command(self, command, properties=None, response=None,
                          timeout=None):
        """ Publish opal command message. Optionally, wait for a response. """
        # pylint: disable=too-many-return-statements
        # pylint: disable=too-many-branches
        # pylint: disable=too-many-statements
        if self._opal_pub is None:
            self._logger.warning("OpalCommand ROS publisher is none!")
            return
        self._logger.info("Sending opal command: " + command)
        # Build message.
        msg = OpalCommand()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Add appropriate command and properties if there are any.
        # We check properties for each command individually, since some
        # require properties, and if there are none, we shouldn't send
        # the message. We assume any properties provided are in the
        # correct format for the command.
        if "RESET" in command:
            msg.command = OpalCommand.RESET
        elif "DISABLE_TOUCH" in command:
            msg.command = OpalCommand.DISABLE_TOUCH
        elif "ENABLE_TOUCH" in command:
            msg.command = OpalCommand.ENABLE_TOUCH
        elif "SIDEKICK_DO" in command:
            msg.command = OpalCommand.SIDEKICK_DO
            # Properties: a string with the name of action to do.
        elif "SIDEKICK_SAY" in command:
            msg.command = OpalCommand.SIDEKICK_SAY
            # Properties: a string with the name of audio file to play.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a " +
                                     "SIDEKICK_SAY command! Not sending " +
                                     " empty command.")
                return
        elif "LOAD_OBJECT" in command:
            msg.command = OpalCommand.LOAD_OBJECT
            # Properties: JSON defining what object to load.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a " +
                                     "LOAD_OBJECT command! Not sending empty" +
                                     " command.")
                return
        elif "CLEAR" in command:
            msg.command = OpalCommand.CLEAR
            # Properties: optionally, string defining what objects to
            # remove.
            if properties:
                msg.properties = properties
        elif "MOVE_OBJECT" in command:
            msg.command = OpalCommand.MOVE_OBJECT
            # Properties: JSON defining what object to move where.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a " +
                                     "MOVE_OBJECT command! Not sending empty" +
                                     " command.")
                return
        elif "HIGHLIGHT" in command:
            msg.command = OpalCommand.HIGHLIGHT_OBJECT
            # Properties: a string with name of the object to highlight.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a " +
                                     "HIGHLIGHT_OBJECT command! Adding null" +
                                     "properties.")
        elif "REQUEST_KEYFRAME" in command:
            msg.command = OpalCommand.REQUEST_KEYFRAME
        elif "FADE_SCREEN" in command:
            msg.command = OpalCommand.FADE_SCREEN
        elif "UNFADE_SCREEN" in command:
            msg.command = OpalCommand.UNFADE_SCREEN
        elif "NEXT_PAGE" in command:
            msg.command = OpalCommand.NEXT_PAGE
        elif "PREV_PAGE" in command:
            msg.command = OpalCommand.PREV_PAGE
        elif "EXIT" in command:
            msg.command = OpalCommand.EXIT
        elif "SET_CORRECT" in command:
            msg.command = OpalCommand.SET_CORRECT
            # Properties: JSON listing names of objects that are
            # correct or incorrect.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a " +
                                     "SET_CORRECT command! Not sending empty" +
                                     " command.")
                return
        elif "SHOW_CORRECT" in command:
            msg.command = OpalCommand.SHOW_CORRECT
        elif "HIDE_CORRECT" in command:
            msg.command = OpalCommand.HIDE_CORRECT
        elif "SETUP_STORY_SCENE" in command:
            msg.command = OpalCommand.SETUP_STORY_SCENE
            # Properties: JSON listing scene attributes.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("Did not get properties for a " +
                                     "SETUP_STORY_SCENE command! Not sending" +
                                     " empty command.")
                return
        else:
            self._logger.warning("Not sending invalid OpalCommand: ", command)
            return
        # Send message.
        self._opal_pub.publish(msg)
        self._logger.debug(msg)

        # If we got a response to wait for and a timeout value, wait
        # for a response.
        if response and timeout:
            self.wait_for_response(response, timeout)

    def send_tega_command(self, command, response=None,
                          timeout=None):
        """ Publish a tega command message and optionally wait for a response.
        """
        if self._tega_pub is None:
            self._logger.warning("TegaAction ROS publisher is none!")
            return
        self._logger.info("Sending Tega command: " + str(command))
        # Build message.
        msg = TegaAction()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # TODO fill in Tega action message. See tega teleop. May want to split.
        # Send message.
        self._tega_pub.publish(msg)
        self._logger.debug(msg)

        # If we got a response to wait for and a timeout value, wait
        # for a response.
        # Timeout should be a datetime.timedelta object.
        if response and timeout:
            self.wait_for_response(response, timeout)

    def send_entrain_audio_message(self, speech, visemes, age, entrain):
        """ Publish EntrainAudio message. """
        if self._entrainer_pub is None:
            self._logger.warning("EntrainAudio ROS publisher is none!")
            return
        print '\nsending entrain speech message: %s' % speech
        msg = EntrainAudio()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.audio = speech
        msg.viseme_file = visemes
        msg.age = age
        msg.entrain = entrain
        self._entrainer_pub.publish(msg)
        rospy.loginfo(msg)

    def send_interaction_state(self, is_user_turn):
        """ Publish an interaction state message. """
        if self._state_pub is None:
            self._logger.warning("InteractionState ROS publisher is none!")
            return
        self._logger.info("Sending interaction state: " + str(is_user_turn))
        # Build message.
        msg = InteractionState()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Set whether it is currently the user's turn in the interaction.
        msg.is_participant_turn = is_user_turn
        # Send message.
        self._state_pub.publish(msg)
        self._logger.debug(msg)

    def on_opal_action_msg(self, data):
        """ Called when we receive OpalAction messages """
        self._logger.info("Received OpalAction message: ACTION=" + data.action
                          + ", MESSAGE=" + data.message + ", OBJECT="
                          + data.objectName)

        # Currently, we are only using OpalAction messages to get
        # responses from the user. So we only care whether the action
        # was a PRESS and whether it was on an object that is used as
        # a START button or is a CORRECT or INCORRECT response
        # object. When we do get one of these messages, set the
        # relevant flag.
        if "tap" in data.action:
            # objectName, position
            pass
        elif "press" in data.action:
            # objectName, position
            # Check if START was in the message.
            if "START" in data.message:
                self.start_response_received = True
                self._response_received = data.message
            # Check if CORRECT was in the message.
        elif "release" in data.action:
            # No object
            pass
        elif "pancomplete" in data.action:
            # No object
            pass
        elif "pan" in data.action:
            # objectName, position
            pass
        elif "collideEnd" in data.action:
            # objectName, position, objectTwoName, positionTwo
            pass
        elif "collide" in data.action:
            # objectName, position, objectTwoName, positionTwo
            pass

    def on_tega_state_msg(self, data):
        """ Called when we receive TegaState messages. """
        # When we get robot state messages, set a flag indicating
        # whether the robot is in motion or playing sound or not.
        self._robot_speaking = data.is_playing_sound
        self._robot_doing_action = data.doing_motion
        self._logger.info("Received TegaState message:"
                          + " doing_motion=" + str(data.doing_motion)
                          + ", motion is=" + str(data.in_motion)
                          + ", playing_sound=" + str(data.is_playing_sound)
                          + ", fidget_set=" + str(data.fidget_set))
        # TODO do something with this? See tega teleop.

    def on_entrainer_msg(self, data):
        """ Called when we receive String log messages from the audio
        entrainer.
        """
        self._logger.info("Receive audio entrainer message: " + str(data))
        # TODO do something with this?

    def wait_for_response(self, response, timeout):
        """ Wait for particular user or robot responses for the
        specified amount of time.
        """
        # Check what response to wait for, set that response received
        # flag to false.
        # Valid responses to wait for are:
        # START, ROBOT_NOT_SPEAKING
        if "START" in response:
            self.start_response_received = False
            self._waiting_for_start = True
            self._waiting_for_robot_speaking = False
        elif "ROBOT_NOT_SPEAKING" in response:
            self._robot_speaking = True
            self._waiting_for_start = False
            self._waiting_for_robot_speaking = True
        else:
            self._logger.warning("Told to wait for " + str(response)
                                 + " but that isn't one of the allowed"
                                 + " responses to wait for!")
            return

        self._logger.info("waiting for " + response + "...")
        start_time = datetime.datetime.now()
        while datetime.datetime.now() - start_time < timeout:
            time.sleep(0.1)
            # Check periodically whether we've received the response we
            # were waiting for, and if so, we're done waiting.
            if (self._waiting_for_start and self.start_response_received) \
                    or (self._waiting_for_robot_speaking
                        and not self._robot_speaking
                        and not self._robot_doing_action):
                self._logger.info("Got " + response + " response!")
                # Reset waiting flags
                self._waiting_for_start = False
                self._waiting_for_robot_speaking = False
                return self._response_received, self._touched_object
        # If we don't get the response we were waiting for, we're done
        # waiting and timed out.
        self._waiting_for_start = False
        self._waiting_for_robot_speaking = False
        self._logger.info("Timed out! Moving on...")
        return "TIMEOUT", ""
