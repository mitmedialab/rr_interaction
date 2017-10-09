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
from asr_google_cloud.msg import AsrResult  # Get ASR results back.
from asr_google_cloud.msg import AsrCommand  # Tell ASR to start/stop.
from std_msgs.msg import Header  # Standard ROS msg header.
from std_msgs.msg import String  # Get string state from audio entrainer.


class RosNode(object):
    """ ROS node: Set up rostopics we publish, subscribe to rostopics
    we care about, functions to send and receive messages.
    """
    # In this case, we need all the instance attributes to hold our ROS
    # publishers, subscribers, and some flags for tracking state.
    # pylint: disable=too-many-instance-attributes

    START = "START"
    ROBOT_NOT_SPEAKING = "ROBOT_NOT_SPEAKING"
    ROBOT_NOT_MOVING = "ROBOT_NOT_MOVING"
    TIMEOUT = "TIMEOUT"
    ROBOT_SPEAKING = "ROBOT_SPEAKING"
    ASR_RESULT = "ASR_RESULT"

    # We keep a dictionary of the strings used in the script and the actual
    # OpalCommand constants to use for lookup.
    OPAL_COMMANDS = {
        "RESET": OpalCommand.RESET,
        "DISABLE_TOUCH": OpalCommand.DISABLE_TOUCH,
        "ENABLE_TOUCH": OpalCommand.ENABLE_TOUCH,
        "SIDEKICK_DO": OpalCommand.SIDEKICK_DO,
        "SIDEKICK_SAY": OpalCommand.SIDEKICK_SAY,
        "LOAD_OBJECT": OpalCommand.LOAD_OBJECT,
        "CLEAR": OpalCommand.CLEAR,
        "MOVE_OBJECT": OpalCommand.MOVE_OBJECT,
        "HIGHLIGHT_OBJECT": OpalCommand.HIGHLIGHT_OBJECT,
        "REQUEST_KEYFRAME": OpalCommand.REQUEST_KEYFRAME,
        "FADE_SCREEN": OpalCommand.FADE_SCREEN,
        "UNFADE_SCREEN": OpalCommand.UNFADE_SCREEN,
        "NEXT_PAGE": OpalCommand.NEXT_PAGE,
        "PREV_PAGE": OpalCommand.PREV_PAGE,
        "EXIT": OpalCommand.EXIT,
        "SET_CORRECT": OpalCommand.SET_CORRECT,
        "SHOW_CORRECT": OpalCommand.SHOW_CORRECT,
        "HIDE_CORRECT": OpalCommand.HIDE_CORRECT,
        "SETUP_STORY_SCENE": OpalCommand.SETUP_STORY_SCENE,
        "STORY_SELECTION": OpalCommand.STORY_SELECTION,
        "SAME_PAGE": OpalCommand.SAME_PAGE,
        "STORY_SHOW_BUTTONS": OpalCommand.STORY_SHOW_BUTTONS,
        "STORY_HIDE_BUTTONS": OpalCommand.STORY_HIDE_BUTTONS,
        "STORY_GO_TO_PAGE": OpalCommand.STORY_GO_TO_PAGE
    }

    def __init__(self, queue):
        """ Initialize ROS """
        # We get a reference to the main node's queue so we can give it
        # messages.
        self._main_queue = queue
        # Set up logger
        self._logger = logging.getLogger(__name__)

        # Initialize the flags we use to track responses from the robot and
        # from the user.
        self._robot_doing_action = False
        self._robot_speaking = False
        self._robot_fidgets = ""
        self._response_received = None
        self._touched_object = ""
        self._start_response_received = False
        self._asr_response_received = False

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
        # Publish commands to tell the ASR node to start or stop processing,
        # as well as what results to publish.
        self._asr_command_pub = rospy.Publisher('asr_command', AsrCommand,
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
        # ASR results from the ASR Google Cloud node.
        rospy.Subscriber('asr_result', AsrResult, self.on_asr_result_msg)

    def send_opal_command(self, command, properties=None):
        """ Publish opal command message. Optionally, wait for a response. """
        if self._opal_pub is None:
            self._logger.warning("OpalCommand ROS publisher is none!")
            return
        self._logger.info("Sending opal command: " + command)
        # Build message.
        msg = OpalCommand()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # We keep a dictionary of the strings used in the script and the actual
        # OpalCommand constants to use for lookup. If we don't find the command
        # given to us in our dictionary, we won't send it. We do, however,
        # print a warning to the user so they can check whether their list of
        # possible commands to send is up-to-date or whether they spelled
        # soemthing wrong.
        try:
            msg.command = self.OPAL_COMMANDS[command]
        except KeyError:
            self._logger.warning("Not sending invalid OpalCommand: ", command)
            return
        # Some commands require properties. If the command is one that requires
        # properties, we shouldn't send the message if we didn't get any
        # properties. If we got properties but the command doesn't need them,
        # we won't add them to the message.
        if (    # Properties: a string with the name of action to do.
                "SIDEKICK_DO" in command
                # Properties: a string with the name of audio file to play.
                or "SIDEKICK_SAY" in command
                # Properties: JSON defining what object to load.
                or "LOAD_OBJECT" in command
                # Properties: JSON defining what object to move where.
                or "MOVE_OBJECT" in command
                # Properties: JSON listing names of objects that are correct or
                # incorrect.
                or "SET_CORRECT" in command
                # Properties: a string with name of the object to highlight.
                or "HIGHLIGHT" in command
                # Properties: JSON listing scene attributes.
                or "SETUP_STORY_SCENE" in command
                # Properties: string name of the story to load next.
                or "STORY_SELECTION" in command):
                # Add properties if we got them. We assume any properties
                # provided are in the correct format for the command.
            if properties:
                msg.properties = properties
            else:
                self._logger.warning("""Did not get properties for
                                     OpalCommand {}! Not sending empty
                                     command.""".format(command))
                return
        # If we have properties for a command that needs them, or no properties
        # for commands that don't, send the message.
        self._opal_pub.publish(msg)
        self._logger.debug(msg)

    def send_tega_command(self, motion="", lookat=None, audio="", fidgets="",
                          enqueue=False, cancel=False, volume=None):
        """ Publish a Tega command message and optionally wait for a response.
        """
        # We may need to send a TegaAction message with any or all of these
        # parameters.
        # pylint: disable=too-many-arguments
        if self._tega_pub is None:
            self._logger.warning("TegaAction ROS publisher is none!")
            return
        self._logger.info("Sending Tega command...")
        # Build message.
        msg = TegaAction()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # If parameters are set, add them to the message...
        msg.motion = motion
        if lookat is not None:
            msg.do_look_at = True
            msg.look_at = lookat
        msg.wav_filename = audio
        msg.fidgets = fidgets
        msg.enqueue = enqueue
        msg.cancel_actions = cancel
        if volume is not None:
            msg.percent_volume = volume
            msg.set_volume = True
        # Send message.
        self._tega_pub.publish(msg)
        self._logger.debug(msg)

    def send_entrain_audio_message(self, speech, visemes):
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
        msg.entrain = True
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

    def send_asr_command(self, command):
        """ Publish an ASR command message to start or stop ASR processing. """
        # Build message with header and command.
        msg = AsrCommand()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # TODO could use some validation to make sure we send a valid command.
        msg.command = command
        # Send message.
        self._asr_command_pub.publish(msg)
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
                self._start_response_received = True
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
        # When we get robot state messages, set flags indicating whether the
        # robot is in motion or playing sound or not, and which fidgets are in
        # use.
        self._robot_speaking = data.is_playing_sound
        self._robot_doing_action = data.doing_motion
        self._robot_fidgets = data.fidget_set
        # TODO temporary
        #self._logger.info("Received TegaState message:"
                          #+ " doing_motion=" + str(data.doing_motion)
                          #+ ", motion is=" + str(data.in_motion)
                          #+ ", playing_sound=" + str(data.is_playing_sound)
                          #+ ", fidget_set=" + str(data.fidget_set))

    def on_entrainer_msg(self, data):
        """ Called when we receive String log messages from the audio
        entrainer.
        """
        self._logger.info("Received audio entrainer message: " + str(data))
        # TODO do something with this?

    def on_asr_result_msg(self, data):
        """ Called when we receive AsrResult messages from the ASR node. """
        self._logger.info("Received ASR message: {}".format(data))
        # Set the response received flag and save the contents of the message
        # so we can return them later.
        self._asr_response_received = True
        self._response_received = (data.transcription.lower(), data.confidence)

    def wait_for_response(self, response, timeout):
        """ Wait for particular user or robot responses for the specified
        amount of time. Timeout should be a datetime.timedelta object.
        """
        # Check what response to wait for, and set that response received flag
        # to false. Valid responses to wait for are defined as constants in
        # this class: START, ROBOT_NOT_SPEAKING, ROBOT_NOT_MOVING,
        # ROBOT_SPEAKING, ASR_RESULT.
        self._response_received = None
        waiting_for_start = False
        waiting_for_robot_not_speaking = False
        waiting_for_robot_not_moving = False
        waiting_for_robot_speaking = False
        waiting_for_asr = False
        if self.START in response:
            self._start_response_received = False
            waiting_for_start = True
        elif self.ROBOT_NOT_SPEAKING in response:
            self._robot_speaking = True
            waiting_for_robot_not_speaking = True
        elif self.ROBOT_NOT_MOVING in response:
            self._robot_doing_action = True
            waiting_for_robot_not_moving = True
        elif self.ROBOT_SPEAKING in response:
            waiting_for_robot_speaking = True
        elif self.ASR_RESULT in response:
            self._asr_response_received = False
            waiting_for_asr = True
        else:
            self._logger.warning("Told to wait for " + str(response)
                                 + " but that isn't one of the allowed"
                                 + " responses to wait for!")
            return

        self._logger.info("waiting for " + response + "...")
        start_time = datetime.datetime.now()
        while datetime.datetime.now() - start_time < timeout:
            time.sleep(0.05)
            # Check periodically to see if we've received the response we were
            # waiting for, and if so, we're done waiting.
            if (waiting_for_start and self._start_response_received) \
                    or (waiting_for_asr and self._asr_response_received) \
                    or (waiting_for_robot_not_speaking
                        and not self._robot_speaking
                        and not self._robot_doing_action) \
                    or (waiting_for_robot_not_moving
                        and not self._robot_doing_action) \
                    or (waiting_for_robot_speaking
                            and self._robot_speaking):
                self._logger.info("Got " + response + " response!")
                return self._response_received, self._touched_object

        # If we don't get the response we were waiting for, we're done
        # waiting and timed out.
        self._logger.info("Timed out! Moving on...")
        return self.TIMEOUT, ""
