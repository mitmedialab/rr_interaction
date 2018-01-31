#!/usr/bin/env python
"""
Jacqueline Kory Westlund
January 2018

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

import os  # For checking if files and directories exist.
import os.path  # For checking if files and directories exist.
import errno  # Used when checking error codes when trying to make a directory.
import datetime  # So we can append the date and time to rosbags.
import toml  # For reading our config file.
import subprocess  # For starting rosbag.
import signal  # Sending SIGINT signal to rosbag.
from src.rr_interaction_node import InteractionHandler  # Start interaction.


def get_args():
    """ Prompt the user to provide the three needed arguments (experimenter
    name, participant ID, and session number) and validate the inputs.
    """
    print "You will need to enter some information to continue launching the" \
          " robot interaction. Please enter the experimenter's name (your " \
          "name, all in lowercase, e.g., jackie), then the current " \
          "participant's ID (e.g., p000), then the current session number " \
          "(e.g., 1)."
    # We need three arguments:
    # 1. The experimenter name (used in the interaction script). We need to
    # validate this
    while True:
        name = raw_input("Enter your name, all lowercase: ")
        # If the corresponding audio file exists, this is an acceptable name.
        if check_name(name):
            print "\tOkay! Hi {}, have fun running the experiment!".format(
                    name)
            break
        else:
            print "{} is not a valid experimenter name. Try again. Example: " \
                  "jackie".format(name)

    # 2. The participant ID (used in the rosbag filename and to start the
    #    interaction with the correct setup). Acceptable IDs will follow the
    #    pattern "p000", "p001", etc.
    while True:
        pid = raw_input("Enter participant ID: ")
        # TODO may want to have a list of valid PIDs that we check against,
        # beyond just checking that the pattern is right.
        if pid.startswith("p") and len(pid) == 4:
            print "\tGreat. Running participant {}.".format(pid)
            break
        else:
            print "{} is not a valid participant ID. Try again. Example: " \
                  "p001".format(pid)

    # 3. The session to play (used to start the interaction with the correct
    #    setup). Acceptable session numbers are positive integers 1-10.
    while True:
        session = raw_input("Enter session number: ")
        try:
            session = int(session)
            if session > 0 and session < 10:
                print "\tSession {}. All set.".format(session)
                break
            else:
                print "{} is not a valid session number. It should be a " \
                      "positive integer between 1-10. Try again. Example: " \
                      "1".format(session)
        except ValueError:
            print "{} is not a valid session number. It should be a positive "\
                  "integer between 1-10. Try again. Example: 1".format(session)

    return (name, pid, session)


def check_name(name):
    """ Check whether a given name exists as an audio file in the audio files
    directory.
    """
    # Read the audio files directory location from the toml config file.
    try:
        with open("config.toml") as tof:
            toml_data = toml.load(tof)
        # Directory of audio files.
        if "audio_base_dir" in toml_data:
            audio_base_dir = toml_data["audio_base_dir"]
            return os.path.isfile(audio_base_dir + "source/" + name + ".wav")
        else:
            print "Could not read audio base directory path! Expected option "\
                  "\"audio_base_dir\" to be in config file. We need to know "\
                  "where the audio files are since we use them to check "\
                  "whether experimenter names are valid."
            exit(1)
    except Exception as exc:  # pylint: disable=broad-except
        print "Could not read the config file \"config.toml\". Exiting " \
              "because we need it to continue, since we use it to check " \
              "whether experimenter names are valid. {}".format(exc)
        exit(1)


def start_rosbag(pid, session):
    """ Start rosbag recording. """
    # Make a directory to save the rosbags in if it doesn't exist yet. If it
    # does exist, move on, unless we get some other error.
    bagdir = "/home/jakory/rr2_rosbags/"
    try:
        os.makedirs(bagdir)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

    # Save rosbag with date appended (that way if the session has to be
    # restarted or if the user enters the wrong ID, any previous bags won't be
    # overwritten).
    date = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    bag = bagdir + pid + "-" + str(session) + "-" + date + ".bag"
    print "Rosbag will be saved to \"{}\"".format(bag)

    # Start rosbag recording.
    try:
        global BAG_PROCESS
        BAG_PROCESS = subprocess.Popen(["rosbag", "record", "-a", "-O", bag],
                                       shell=False)
    except:
        print "Uh oh, something went wrong with rosbag recording!"
        raise


if __name__ == '__main__':
    """ Start the relational robot interaction node and rosbag recording. This
    cannot be done in the roslaunch file because the configuration of the node
    and the rosbag changes with each participant. If these were included in the
    roslaunch file, we would have to restart everything to change the
    participant, which is a lot of restarting. Plus, if roscore is started from
    the roslaunch file, then both the robot and tablet would also need to be
    restarted. So, instead, we get arguments from the user, try starting the
    rosbag recording, and try launching the interaction.
    """
    try:
        # Get input from the user.
        (EXPERIMENTER, PARTICIPANT, SESSION) = get_args()
        # Start the rosbag recording.
        BAG_PROCESS = None
        start_rosbag(PARTICIPANT, SESSION)
        # Start the interaction with the provided session number, participant
        # ID, entrainment set to true, and experimenter name.
        INTERACTION_HANDLER = InteractionHandler()
        INTERACTION_HANDLER.launch_interaction(SESSION, PARTICIPANT, True,
                                               EXPERIMENTER)

    except Exception as exc:
        print "Uh oh, something went wrong! {}".format(exc)
        print "Stopping rosbag recording if it's not stopped already..."
        BAG_PROCESS.send_signal(signal.SIGINT)
        BAG_PROCESS.wait()
        raise

    # After the interaction ends, stop rosbag recording.
    print "Stopping rosbag recording..."
    BAG_PROCESS.send_signal(signal.SIGINT)
    BAG_PROCESS.wait()
