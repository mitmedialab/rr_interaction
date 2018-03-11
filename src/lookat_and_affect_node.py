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

import atexit  # Handle cleanup before exit.
import logging  # Log messages.
import random  # For choosing actions.
import rospy  # ROS
import threading  # Timer for action callbacks.
import rr_commons as rr_commons  # Common constants, such as lookat vectors.
from affdex_msgs.msg import AffdexFrameInfo  # Receive affdex data.
from rr_msgs.msg import InteractionState  # Get info on the interaction state.
from r1d1_msgs.msg import TegaAction  # Send commands to Tega.
from sar_opal_msgs.msg import OpalCommand  # See commands sent to Opal devices.
from std_msgs.msg import Header  # Standard ROS msg header.


POSITIVE_ANIMS = [TegaAction.MOTION_AGREEMENT, TegaAction.MOTION_LAUGH,
                  TegaAction.MOTION_INTERESTED, TegaAction.MOTION_SMILE,
                  TegaAction.MOTION_SHIMMY, TegaAction.MOTION_YES]
NEGATIVE_ANIMS = [TegaAction.MOTION_PUZZLED, TegaAction.MOTION_THINKING,
                  TegaAction.MOTION_SIDEPERK]
THINKING_ANIMS = [TegaAction.MOTION_PUZZLED, TegaAction.MOTION_THINKING,
                  TegaAction.MOTION_THINK1, TegaAction.MOTION_THINK2,
                  TegaAction.MOTION_THINK3]
HAPPY_ANIMS = [TegaAction.MOTION_SILENT_AGREEMENT,
               TegaAction.MOTION_SILENT_LAUGH, TegaAction.MOTION_INTERESTED,
               TegaAction.MOTION_HAPPY_UP, TegaAction.MOTION_SMILE,
               TegaAction.MOTION_SILENT_HAPPY_WIGGLE, TegaAction.MOTION_YES,
               TegaAction.MOTION_SILENT_HAPPY_DANCE]


def on_state_msg(data):
    """ When we receive an interaction state message, perform lookat behaviors
    as appropriate.
    """
    LOGGER.info("Got state message: {}".format(data.state))
    global USER_STORY, USER_TURN, RELATIONAL, ROBOT_SLEEPING

    # If the state message tells us about the condition, save it so we know
    # whether to respond or not.
    if "RR" in data.state:
        RELATIONAL = True

    if "robot sleeping" in data.state:
        ROBOT_SLEEPING = True

    # If it is the user's turn to speak or act, we should mostly look at them.
    # Elsewhere, the robot is told to look at the user when it is the user's
    # turn to speak.
    if data.is_participant_turn:
        USER_TURN = True
        return
    else:
        USER_TURN = False

    # If this is during a story or a picture task, we can occasionally glance
    # at the tablet. We will also be backchanneling.
    if ("start child story retell" in data.state or
            "start child story create" in data.state or
            "start picture task" in data.state):
        # Look at the tablet and start alternating.
        send_tega_command(lookat=rr_commons.LOOKAT["TABLET"])
        USER_STORY = True
        do_story_lookat(False)
    elif ("end child story retell" in data.state or
            "end child story create" in data.state or
            "end picture task" in data.state):
        # At the end of the user's story, stop glancing at the tablet.
        USER_STORY = False


def on_opal_msg(data):
    """ Use OpalCommand messages to know whether to look at the tablet. """
    LOGGER.info("Got opal command message: {}".format(data.command))
    # If the tablet page is told to change or something new is loaded on the
    # tablet, look at the tablet for a few seconds, then back at the child.
    if (data.command == OpalCommand.NEXT_PAGE or
            data.command == OpalCommand.PREV_PAGE or
            data.command == OpalCommand.STORY_SELECTION or
            data.command == OpalCommand.STORY_SHOW_BUTTONS or
            data.command == OpalCommand.LOAD_OBJECT or
            data.command == OpalCommand.STORY_GO_TO_PAGE):

        # The non-relational robot will look around randomly, at approximately
        # the same intervals as the relational robot.
        if not RELATIONAL:
            LOGGER.info("NR: Sending random lookat")
            send_tega_command(lookat=rr_commons.LOOKAT[rr_commons.LOOKAT_ARR[
                              random.randint(
                                  0, len(rr_commons.LOOKAT_ARR) - 1)]])
            # Call this function again in a little while to look again. Use a
            # random distribution that means the robot will look somewhere
            # again with approximately the same frequency as the relational
            # robot.
            threading.Timer(2.5 + random.uniform(-0.5, 0.5),
                            send_tega_command(lookat=rr_commons.LOOKAT[
                                rr_commons.LOOKAT_ARR[random.randint(
                                    0,
                                    len(rr_commons.LOOKAT_ARR) - 1)]])).start()
        else:
            LOGGER.info("RR: Sending lookat tablet then user")
            # The relational robot will look at the tablet.
            send_tega_command(lookat=rr_commons.LOOKAT["TABLET"])
            # Wait for a few seconds, then look back at the child.
            threading.Timer(2.5 + random.uniform(-0.5, 0.5),
                            send_tega_command(
                                lookat=rr_commons.LOOKAT["USER"])).start()


def do_story_lookat(looking_at_user):
    """ During a story, look between the user and the tablet. """
    # If the user is no longer telling a story, stop sending lookats.
    if not USER_STORY:
        return

    # The non-relational robot will look around randomly, at approximately the
    # same intervals as the relational robot.
    if not RELATIONAL:
        LOGGER.info("NR: Sending random lookat")
        send_tega_command(lookat=rr_commons.LOOKAT[rr_commons.LOOKAT_ARR[
            random.randint(0, len(rr_commons.LOOKAT_ARR) - 1)]])
        # Call this function again in a little while to look back at the user.
        # Use a random distribution that means the robot will look somewhere
        # again with approximately the same frequency as the relational robot.
        threading.Timer(3.5 + random.uniform(-1.0, 2.0),
                        do_story_lookat(False)).start()

    # When the child is telling or retelling a story, look at the tablet most
    # of the time, but glance at the child sometimes. Look at the child about
    # every 5s, for about 2.5s. Then look back at the tablet. These intervals
    # are what were used in the SR2 story study during child stories.
    if looking_at_user:
        LOGGER.info("RR: Sending lookat tablet")
        send_tega_command(lookat=rr_commons.LOOKAT["TABLET"])
        # Call this function again in a little while to look back at the user.
        threading.Timer(5 + random.uniform(-0.5, 0.5),
                        do_story_lookat(False)).start()
    else:
        LOGGER.info("RR: Sending lookat user")
        send_tega_command(lookat=rr_commons.LOOKAT["USER"])
        # Call this function again in a little while to look back at the
        # tablet.
        threading.Timer(2.5 + random.uniform(-0.5, 0.5),
                        do_story_lookat(True)).start()


def send_tega_command(lookat=None, motion=None):
    """ Publish a Tega command message containing a lookat command or a motion
    command.
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
    # Add the lookat command and/or motion command.
    if lookat is not None:
        msg.do_look_at = True
        msg.look_at = lookat
    if motion is not None:
        msg.motion = motion
    # Queue the action so it doesn't play over or interrupt anything.
    msg.enqueue = True
    # Send message.
    TEGA_PUB.publish(msg)
    LOGGER.info(msg)


def on_affdex_msg(data):
    """ When we receive Affdex data, process it and determine what behaviors to
    do, if any.
    """
    global COUNTER
    COUNTER += 1

    # The non-relational robot gets some random animations to play, but only
    # when it is the user's turn, not during a user story, the robot is not
    # sleeping.
    if not RELATIONAL and USER_TURN and not USER_STORY and not ROBOT_SLEEPING \
            and COUNTER >= 90:
        LOGGER.info("NR: Checking to see if we do random affect anim yet...")
        # A small amount of the time send an animation. Randomly pick between
        # sending positive and negative ones.
        if random.random() < 0.1:
            LOGGER.info("NR: Doing random affect anim!")
            motion = POSITIVE_ANIMS[random.randint(
                    0, len(POSITIVE_ANIMS) - 1)] if random.random() < 0.5 \
                else NEGATIVE_ANIMS[random.randint(
                    0, len(NEGATIVE_ANIMS) - 1)]
            send_tega_command(motion=motion)
        else:
            LOGGER.info("NR: Nope, not doing random affect anim!")
        COUNTER = 0
        return

    # The relational robot will respond reactively during appropriate states of
    # the interaction. We want to respond to:
    #  - positive smiles/laughs with smiles/laughs/nods.
    #  - lack of attention with animations that gain attention.
    #  - some emotions/valence during child's turn to speak.
    # Keep a running array of recent frames for these various metrics so we can
    # use them to adapt the robot's behavior.
    VALENCE.append(data.emotions[8])
    FACE_DETECTED.append(data.face_detected)
    SMILES.append(data.expressions[12])
    ATTENTION.append(data.attention_detected)
    DISTANCE.append(data.face_distance)
    THINKING.append(data.expressions[20])
    SAD.append(data.emotions[6])
    HAPPY.append(data.emotions[0])
    SURPRISE.append((data.emotions[7], data.expressions[2]))

    # User stories have backchanneling so let's not add more stuff during them.
    # If we are not in a user story and the robot is not supposed to be
    # sleeping, respond! If we have 90+ values in our array (~3s of data at
    # 30fps), we can process it and see what to do.  This is the number the
    # storyteller study used so we're using the same value here.
    if not USER_STORY and not ROBOT_SLEEPING and COUNTER >= 90:
        LOGGER.info("RR: Time to react to affect!")
        react_to_affect()


def react_to_affect():
    """ Time to react to the collected affect data! """
    global FACE_DETECTED, VALENCE, SMILES, ATTENTION, THINKING, SAD, COUNTER, \
        DISTANCE, HAPPY, SURPRISE
    LOGGER.info("Checking to see if we should react to affect...")

    # If the user suddenly leaned in or leaned out, have the robot mirror that
    # change in distance.
    lean = user_leaned()
    if "IN" in lean:
        LOGGER.info("Leaned in! Sending action...")
        send_tega_command(motion=TegaAction.MOTION_POSE_LEAN_FORWARD)
    elif "OUT" in lean:
        LOGGER.info("Leaned out! Sending action...")
        send_tega_command(motion=TegaAction.MOTION_SILENT_SCARED)

    # Everything else should only happen on the user's turn.
    if USER_TURN:
        # If the user smiled positively (as opposed to a frustrated/negative
        # smile), have the robot smile two thirds of the time, and otherwise
        # nod or laugh.
        if user_is_smiling() and valence_is_positive():
            LOGGER.info("Smiling! Sending action...")
            if random.randint(1, 6) > 4:
                send_tega_command(motion=TegaAction.MOTION_SMILE)
            elif random.random() < 0.5:
                send_tega_command(motion=TegaAction.MOTION_SILENT_NOD)
            else:
                send_tega_command(motion=TegaAction.MOTION_LAUGH)

        # If the user is not attending, play an animation to regain attention.
        # Play different animations if the user shows positive vs. negative
        # valence.
        elif not user_is_attending():
            LOGGER.info("Not attending! Sending action...")
            motion = POSITIVE_ANIMS[random.randint(
                    0, len(POSITIVE_ANIMS) - 1)] if valence_is_positive() \
                else NEGATIVE_ANIMS[random.randint(
                    0, len(NEGATIVE_ANIMS) - 1)]
            send_tega_command(motion=motion)

        # See if there is other affect to mirror while the user is speaking.
        # Thinking, puzzled (lid tighten)
        elif user_is_thinking():
            LOGGER.info("Thinking! Sending action...")
            send_tega_command(motion=THINKING_ANIMS[
                random.randint(0, len(THINKING_ANIMS) - 1)])
        # Sad
        elif user_is_sad():
            LOGGER.info("Sad! Sending action...")
            send_tega_command(motion=TegaAction.MOTION_SAD)

        # Surprise, brow raise
        elif user_is_surprised():
            LOGGER.info("Surprised! Sending action...")
            send_tega_command(motion=TegaAction.MOTION_SILENT_SCARED)

        # Happy/joy
        elif user_is_happy():
            LOGGER.info("Happy! Sending action...")
            send_tega_command(motion=HAPPY_ANIMS[
                random.randint(0, len(HAPPY_ANIMS) - 1)])

    # Clear arrays so we process new data next time.
    COUNTER = 0
    VALENCE = []
    FACE_DETECTED = []
    SMILES = []
    ATTENTION = []
    DISTANCE = []
    THINKING = []
    SAD = []
    HAPPY = []
    SURPRISE = []


def valence_is_positive():
    """ Check the array of collected valence measurements to see if it is
    overall positive or negative.
    """
    face_count = sum(i is True for i in FACE_DETECTED)
    pos_count = sum(i > 0 for i in VALENCE)
    neg_count = sum(i < 0 for i in VALENCE)
    LOGGER.info("faces: {}, pos: {}, neg: {}".format(face_count, pos_count,
                                                      neg_count))
    # If positive valence was detected in more that 65% of the faces detected
    # in recent frames, say we are overall positive.
    if pos_count >= 0.55 * face_count:
        return True
    # If negative valence was detected in more than 25% of the faces detected
    # in recent frames, say we are overall negative.
    if neg_count >= 0.15 * face_count:
        return False


def user_is_attending():
    """ Check the array of collected attention measurements to see if the user
    is generally attentive or not.
    """
    face_count = sum(i is True for i in FACE_DETECTED)
    atten_count = sum(i is True for i in ATTENTION)
    # Count the user as attentive if they are displaying attention in 60% of
    # the recent frames with faces in them or more.
    LOGGER.info("faces: {}, attention: {}".format(face_count, atten_count))
    return atten_count >= 0.4 * face_count


def user_is_smiling():
    """ Check the array of collected smile scores to see if the user is
    smiling or not.
    """
    # If even a small percentage of the recent frames with faces had a smile,
    # count it as a smile.
    face_count = sum(i is True for i in FACE_DETECTED)
    smile_count = sum(i > 30 for i in SMILES)
    LOGGER.info("faces: {}, smiles: {}".format(face_count, smile_count))
    return smile_count >= 0.02 * face_count


def user_is_thinking():
    """ Check the array of collected thinking-related scores to see if the user
    is showing a thinking expression or not.
    """
    # If 20% or more of the recent frames had faces with a thinking/lid tighten
    # expression, count it as thinking.
    face_count = sum(i is True for i in FACE_DETECTED)
    think_count = sum(i > 30 for i in THINKING)
    LOGGER.info("faces: {}, thinks: {}".format(face_count, think_count))
    return think_count >= 0.1 * face_count


def user_is_sad():
    """ Check the array of collected sad scores to see if the user is showing a
    sad expression or not.
    """
    # If 10% or more of the recent frames had faces with a sad expression,
    # count it as sad.
    face_count = sum(i is True for i in FACE_DETECTED)
    sad_count = sum(i > 10 for i in SAD)
    LOGGER.info("faces: {}, sad: {}".format(face_count, sad_count))
    return sad_count >= 0.1 * face_count


def user_is_happy():
    """ Check the array of collected happy scores to see if the user is showing
    a happy/joyful expression or not.
    """
    # If 20% or more of the recent frames had faces with a happy expression,
    # count it as happy.
    face_count = sum(i is True for i in FACE_DETECTED)
    happy_count = sum(i >= 40 for i in HAPPY)
    LOGGER.info("faces: {}, happy: {}".format(face_count, happy_count))
    return happy_count >= 0.1 * face_count


def user_is_surprised():
    """ Check the array of collected surprise-related scores to see if the user
    is showing a surprised expression or not.
    """
    # If 10% or more of the recent frames had faces with a surprise expression,
    # count it as surprised. The SURPRISE list is a list of tuples of values:
    # (surprise, brow raise) because both are relevant.
    face_count = sum(i is True for i in FACE_DETECTED)
    surprise_count = sum(i[0] >= 25 or i[1] >= 50 for i in SURPRISE)
    LOGGER.info("faces: {}, surprise: {}".format(face_count, surprise_count))
    return surprise_count >= 0.05 * face_count


def user_leaned():
    """ Use the array of collected estimated distances from the robot/camera to
    see if the user suddenly leaned in or out.
    """
    # According TegaCam-Affect-Analysis, the face distance is best at 60. The
    # range is from 30-90. No units are specified. Probably centimeters?
    # We look at the deltas between each pair of frames. We can see in general
    # if the user is leaning in or leaning out between frames by looking at the
    # sign of the delta (positive means the distance is growing, i.e., leaning
    # out; negative means the distance is shrinking, i.e., leaning in). We can
    # add up the deltas to get the overall change in distance. We can look at
    # the size of the deltas to see if there was a big change or not.
    delta_in = 0
    delta_out = 0
    for i in range(1, len(DISTANCE)):
        delta = DISTANCE[i] - DISTANCE[i - 1]
        if delta < 0:
            LOGGER.info("leaned in: {}".format(delta))
            delta_in += delta
        if delta > 0:
            LOGGER.info("leaned out: {}".format(delta))
            delta_out += delta
        else:
            LOGGER.info("no change: {}".format(delta))

    # Return "IN" if the user leaned in; "OUT" if the user leaned out, nothing
    # if there was no big change in lean.
    LOGGER.info("total delta in: {}, total delta out: {}".format(
        delta_in, delta_out))
    if delta_in < -5:
        return "IN"
    if delta_out > 5:
        return "OUT"
    return ""


def cleanup():
    """ Cleanup before exit. """
    LOGGER.info("Cleaning up lookat handler...")
    if ISTATE_SUB:
        ISTATE_SUB.unregister()
    if OPAL_SUB:
        OPAL_SUB.unregister()
    if AFFDEX_SUB:
        AFFDEX_SUB.unregister()


if __name__ == "__main__":
    """ Send lookat commands to the robot based on the current interaction
    state.
    """
    # Set up logger.
    LOGGER = logging.getLogger(__name__)
    LOGGER.info("Initializing lookat handler...")

    # Add exit handler.
    atexit.register(cleanup)

    try:
        # Initialize our ROS node.
        rospy.init_node("lookat_handler", anonymous=False)

        LOGGER.info("We will publish to topics: /tega")
        LOGGER.info("We subscribe to topics: /rr/state, /rr/opal_command")

        # We will send action or speech commands to the Tega robot.
        TEGA_PUB = rospy.Publisher('/tega', TegaAction, queue_size=10)

        # Interaction state messages.
        ISTATE_SUB = rospy.Subscriber('/rr/state', InteractionState,
                                      on_state_msg)

        # Opal Command messages, so we can see when the tablet page changes.
        OPAL_SUB = rospy.Subscriber('/rr/opal_command', OpalCommand,
                                    on_opal_msg)

        # Affdex data messages.
        AFFDEX_SUB = rospy.Subscriber('/affdex_data', AffdexFrameInfo,
                                      on_affdex_msg)

        # We do not start in a user story or on the user's turn to speak.
        USER_STORY = False
        USER_TURN = False
        # We need to track if the robot is supposed to be sleeping so that we
        # don't respond if it is.
        ROBOT_SLEEPING = False

        # We do not start by responding to affect data. We will only respond if
        # the user is in the correct (relational) condition.
        RELATIONAL = False

        # For tracking recent affdex data.
        VALENCE = []
        FACE_DETECTED = []
        SMILES = []
        ATTENTION = []
        DISTANCE = []
        THINKING = []
        HAPPY = []
        SAD = []
        SURPRISE = []
        COUNTER = 0

        # Spin.
        rospy.spin()

    # If roscore isn't running or shuts down unexpectedly, we won't continue.
    except rospy.ROSInterruptException:
        print "ROS node shutdown"
