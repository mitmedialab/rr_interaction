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
import random  # For choosing backchannel actions.
import time  # For sleep.
import threading  # Timer for randomly backchanneling.
import logging  # For log messages.
from r1d1_msgs.msg import TegaAction  # Send commands to Tega.
from r1d1_msgs.msg import TegaState  # ROS msgs to get info from Tega.
from sar_opal_msgs.msg import OpalCommand  # Send commands an Opal device.
from sar_opal_msgs.msg import OpalAction  # Get Opal device state and actions.
from rr_msgs.msg import EntrainAudio  # Send audio to the audio entrainer.
from rr_msgs.msg import InteractionState  # Send state to the audio entrainer.
from rr_msgs.msg import UserInput  # Receive user input form responses.
from asr_google_cloud.msg import AsrResult  # Get ASR results back.
from asr_google_cloud.msg import AsrCommand  # Tell ASR to start/stop.
from std_msgs.msg import Header  # Standard ROS msg header.
from std_msgs.msg import String  # Several different ROS msgs.


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
    ASR_OR_TABLET = "ASR_OR_TABLET"
    TABLET_RESPONSE = "TABLET_RESPONSE"
    USER_INPUT_NEGOTIATION = UserInput.NEGOTIATION
    USER_INPUT_INTERACTION_CONTROL = UserInput.INTERACTION_CONTROL

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

    def __init__(self, queue, use_entrainer, entrain, audio_base_dir,
                 viseme_base_dir, backchannel_actions, backchannel_random):
        """ Initialize ROS """
        # Set up logger
        self._logger = logging.getLogger(__name__)
        # We get a reference to the main node's queue so we can give it
        # messages.
        self._main_queue = queue
        # Backchanneling starts disabled.
        self._backchanneling_enabled = False
        self._story_backchanneling_enabled = False
        self._backchannel_random = False
        # Do we send audio through the entrainer or not? Does the entrainer
        # actually entrain that audio, or merely stream it?
        self._use_entrainer = use_entrainer
        self._entrain = entrain
        # Filepaths for audio and visemes, if we need them for sending through
        # the entrainer.
        self._audio_base_dir = audio_base_dir
        self._viseme_base_dir = viseme_base_dir
        # These are different actions that could be taken for different
        # backchannel states that can occur.
        self._backchannel_actions = backchannel_actions

        # Initialize the flags we use to track responses from the robot and
        # from the user.
        self._robot_doing_action = False
        self._robot_speaking = False
        self._robot_fidgets = ""
        self._response_received = None
        self._start_response_received = False
        self._asr_response_received = False
        self._control_response_received = False
        self._tablet_response_received = None
        self._negotiation_response_received = False

        # Set up rostopics we publish:
        self._logger.info("We will publish to topics: rr/opal_command, /tega, "
                          + "/rr/state"
                          + (", /rr/entrain_audio" if self._use_entrainer
                             else ""))

        # Commands to the Opal game.
        self._opal_pub = rospy.Publisher('/rr/opal_command', OpalCommand,
                                         queue_size=10)
        # Commands to the Tega robot.
        self._tega_pub = rospy.Publisher('/tega', TegaAction, queue_size=10)

        # Commands to entrain audio, if entrainment is enabled.
        self._entrainer_pub = None
        if self._use_entrainer:
            self._entrainer_pub = rospy.Publisher('/rr/entrain_audio',
                                                  EntrainAudio,
                                                  queue_size=10)
        # Interaction state messages.
        self._state_pub = rospy.Publisher('/rr/state', InteractionState,
                                          queue_size=10)
        # Publish commands to tell the ASR node to start or stop processing,
        # as well as what results to publish.
        self._asr_command_pub = rospy.Publisher('/asr_command', AsrCommand,
                                                queue_size=10)

        # Set up rostopics we subscribe to:
        self._logger.info("Subscribing to topics: /rr/opal_action, "
                          + "/tega_state, /asr_result, /msg_bc"
                          + (", /rr/audio_entrainer" if self._use_entrainer
                             else ""))
        # State from the Opal game.
        rospy.Subscriber('/rr/opal_action', OpalAction,
                         self.on_opal_action_msg)
        # State from the Tega robot.
        rospy.Subscriber('/tega_state', TegaState, self.on_tega_state_msg)
        # ASR results from the ASR Google Cloud node.
        rospy.Subscriber('/asr_result', AsrResult, self.on_asr_result_msg)
        # User input form responses.
        rospy.Subscriber('/rr/user_input', UserInput, self.on_user_input_msg)
        # Subscribe to backchannel output so we know when to backchannel and
        # what kind of action to do.
        rospy.Subscriber("msg_bc", String, self.on_bc_msg_received)

        if self._use_entrainer:
            # State from the audio entrainer.
            rospy.Subscriber('/rr/audio_entrainer', String,
                             self.on_entrainer_msg)

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
                self._logger.warning("Did not get properties for OpalCommand "
                                     "{}! Not sending empty command.".format(
                                         command))
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

    def send_entrain_audio_message(self, speech, visemes, entrain):
        """ Publish EntrainAudio message. """
        if self._entrainer_pub is None:
            self._logger.warning("EntrainAudio ROS publisher is none!")
            return
        self._logger.debug("Sending entrain speech message: {}".format(speech))
        msg = EntrainAudio()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.audio = speech
        msg.viseme_file = visemes
        msg.entrain = entrain
        self._entrainer_pub.publish(msg)
        self._logger.debug(msg)

    def send_speech(self, audio):
        """ Send audio either through the entrainer or directly to the robot.
        """
        if self._use_entrainer:
            # Send the filename to the audio entrainer. Append the filepath to
            # the filename before sending. Note that an empty filepath can be
            # provided if the full filepaths are given in the script. We assume
            # that corresponding viseme files have the same name but with a
            # .txt extension, and are located at the viseme filepath. If full
            # filepaths are provided in the script, then an empty filepath
            # should be provided for the viseme filepath as well, and the
            # viseme text files should be located in the same directory as the
            # audio.
            self.send_entrain_audio_message(
                self._audio_base_dir + audio,
                self._viseme_base_dir + audio.replace(".wav", ".txt"),
                self._entrain)
        else:
            # Send directly to the robot.
            self.send_tega_command(audio=audio, enqueue=True)

    def send_interaction_state(self, is_user_turn=False, state=""):
        """ Publish an interaction state message. """
        if self._state_pub is None:
            self._logger.warning("InteractionState ROS publisher is none!")
            return
        self._logger.info("Sending interaction state: \n\tis user turn? {}\n\t"
                          "state: {}".format(is_user_turn, state))
        # Build message.
        msg = InteractionState()
        # Add header.
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # Set whether it is currently the user's turn in the interaction.
        msg.is_participant_turn = is_user_turn
        # Send the current state, if any.
        msg.state = state
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
            # Check if we got a press on some other named object.
            elif data.objectName:
                self._tablet_response_received = (data.objectName,
                                                  data.position)
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

    def on_user_input_msg(self, data):
        """ Called when we receive UserInput messages from the user input form.
        """
        self._logger.info("Received UserInput message: {}".format(data))
        # Set the response received flag and save the contents of the message
        # so we can return them later.
        if self.USER_INPUT_NEGOTIATION in data.response_type:
            self._negotiation_response_received = True
        if self.USER_INPUT_INTERACTION_CONTROL in data.response_type:
            self._control_response_received = True
            # Tell the main thread when we receive this, in case it means we
            # need to start, stop, pause, or resume the interaction.
            self._main_queue.put(data.response)
        self._response_received = data.response

    def enable_backchanneling(self, enabled, story=False):
        """ Turn backchanneling on or off. """
        self._backchanneling_enabled = enabled
        # If the story flag was set, then we will sometimes use story prompts
        # in our backchannel action set.
        self._story_backchanneling_enabled = story
        if enabled and self._backchannel_random:
            self._randomly_backchannel()

    def randomly_backchannel(self):
        """ Randomly play backchannel actions every so often until
        backchanneling is disabled.
        """
        # If backchanneling is no longer enabled, we'll stop sending actions.
        if not self._backchanneling_enabled:
            return

        if "random" not in self._backchannel_actions:
            self._logger.warning("No random backchannel actions listed in the "
                                 "script config!")
            return

        # Randomly pick a backchannel action to do and send it to the robot.
        self._logger.info("Choosing a random backchannel action...")
        command = self._backchannel_actions["random"]["actions"][
                    random.randint(0, len(self._backchannel_actions[
                        "random"]["actions"]) - 1)]
        # If the command is lowercase, it's speech; otherwise, it's an
        # animation/motion command that we should send directly to the robot.
        if command.islower():
            self.send_speech(command + ".wav")
        else:
            self.send_tega_command(motion=command)

        # Call this function again to randomly backchannel again in a little
        # while, after a random interval of seconds (interval from Park et al.
        # 2017 backchanneling HRI paper).
        threading.Timer(5.53 + random.uniform(-1.5, 1.5),
                randomly_backchannel).start()

    def on_bc_msg_received(self, data):
        """ When we receive output from the backchannel module, if
        backchanneling is enabled, send an appropriate action command to the
        robot, or, if it is in use and the action to take is speech, through
        the audio entrainer.
        """
        if not self._backchanneling_enabled and not self._backchannel_random:
            return

        # If backchanneling is happening during a story, we can occasionally
        # use a story prompt after a long pause instead of a regular
        # backchannel action.
        if self._story_backchanneling_enabled and \
                "long_pause" in data.data.lower() and \
                random.randint(1, 5) > 3:
            self._logger.info("Choosing a story backchannel action...")
            command = self._backchannel_actions["long_pause_story"]["actions"][
                    random.randint(0, len(self._backchannel_actions[
                        "long_pause_story"]["actions"]) - 1)]

        # Randomly select a backchannel action based on the type of action
        # we should take and send to the robot.
        command = ""
        if data.data.lower() in self._backchannel_actions:
            self._logger.info("Choosing {} backchannel action...".format(
                data.data))
            command = self._backchannel_actions[data.data.lower()]["actions"][
                    random.randint(0, len(self._backchannel_actions[
                        data.data.lower()]["actions"]) - 1)]

        # If the command is lowercase, it's speech; otherwise, it's an
        # animation/motion command that we should send directly to the robot.
        if command.islower():
            self.send_speech(command + ".wav")
        else:
            self.send_tega_command(motion=command)

    def wait_for_response(self, response, timeout):
        """ Wait for particular user or robot responses for the specified
        amount of time. Timeout should be a datetime.timedelta object.
        """
        # Check what response to wait for, and set that response received flag
        # to false. Valid responses to wait for are defined as constants in
        # this class.
        self._response_received = None
        waiting_for_start = False
        waiting_for_robot_not_speaking = False
        waiting_for_robot_not_moving = False
        waiting_for_robot_speaking = False
        waiting_for_asr = False
        waiting_for_negotiation = False
        waiting_for_control = False
        waiting_for_tablet = False
        if self.START in response:
            self._start_response_received = False
            waiting_for_start = True
        elif self.ROBOT_NOT_SPEAKING in response:
            # Wait for the robot to start speaking before we go on to wait for
            # the robot to be done speaking.
            self.wait_for_speaking()
            waiting_for_robot_not_speaking = True
        elif self.ROBOT_NOT_MOVING in response:
            # Wait for the robot to start moving before we go on to wait for it
            # to be done moving.
            self.wait_for_motion()
            waiting_for_robot_not_moving = True
        elif self.ROBOT_SPEAKING in response:
            waiting_for_robot_speaking = True
        elif self.ASR_RESULT in response:
            self._asr_response_received = False
            waiting_for_asr = True
        elif self.USER_INPUT_NEGOTIATION in response:
            self._negotiation_response_received = False
            waiting_for_negotiation = True
        elif self.USER_INPUT_INTERACTION_CONTROL in response:
            self._control_response_received = False
            waiting_for_control = True
        elif self.TABLET_RESPONSE in response:
            self._tablet_response_received = None
            waiting_for_tablet = True
        elif self.ASR_OR_TABLET in response:
            self._tablet_response_received = None
            waiting_for_tablet = True
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
            #
            # We return the response type only when it is otherwise ambiguous,
            # such as if we are waiting for ASR_OR_TABLET. Also note that
            # placing the tablet stuff here effectively means that tablet
            # responses take priority over ASR responses if we get both
            # responses.
            if waiting_for_tablet and self._tablet_response_received:
                self._logger.info("Got {} response!".format(response))
                return self._tablet_response_received, self.TABLET_RESPONSE
            elif (waiting_for_start and self._start_response_received) \
                    or (waiting_for_asr and self._asr_response_received) \
                    or (waiting_for_negotiation and
                        self._negotiation_response_received) \
                    or (waiting_for_control and
                        self._control_response_received) \
                    or (waiting_for_robot_not_speaking
                        and not self._robot_speaking
                        and not self._robot_doing_action) \
                    or (waiting_for_robot_not_moving
                        and not self._robot_doing_action) \
                    or (waiting_for_robot_speaking
                        and self._robot_speaking):
                self._logger.info("Got {} response!".format(response))
                return self._response_received, ""

        # If we don't get the response we were waiting for, we're done
        # waiting and timed out.
        self._logger.info("Timed out! Moving on...")
        return self.TIMEOUT, ""

    def wait_for_not_speaking(self):
        """ Wait until the robot is not making any sound. """
        while self._robot_speaking:
            time.sleep(0.05)

    def wait_for_speaking(self, timeout=12):
        """ Wait until we hear the robot start playing sound before going on to
        process the next command and wait for the robot to be done playing
        sound. We have to wait because when streaming audio through the audio
        entrainer, it sometimes takes a couple seconds for the audio to be
        processed and sent to the robot. So we want to make sure we wait until
        the robot has gotten the command to play audio before we move on to the
        next item in the script. Otherwise, we might see that the robot isn't
        playing any sound and send the next item in the script too soon,
        clobbering the audio that's about to be played as it is sent from the
        entrainer to the robot.
        """
        counter = 0
        increment = 0.05
        while not self._robot_speaking and counter < timeout:
            counter += increment
            time.sleep(increment)

        self._logger.debug("Waited {} seconds for robot to start sound".format(
            counter))
        if counter >= timeout:
            self._logger.warning("Warning: timed out waiting for robot to "
                                 "start playing sound! timeout: {} "
                                 "Moving on...".format(timeout))

    def wait_for_motion(self, timeout=5):
        """ Wait until the robot has started playing an animation before going
        on to wait for the robot to be done playing it (similar to waiting for
        sound, above).
        """
        # TODO Could possibly combine this with wait_for_speaking and pass in
        # what to wait for.
        counter = 0
        increment = 0.05
        while not self._robot_doing_action and counter < timeout:
            counter += increment
            time.sleep(increment)

        if counter >= timeout:
            self._logger.warning("Warning: timed out waiting for robot to "
                                 "start doing motion! timeout: {} "
                                 "Moving on...".format(timeout))
