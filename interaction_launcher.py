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
import psutil  # Dealing with closing the rosbag properly.
import signal  # Sending SIGINT signal to rosbag.
from src.rr_interaction_node import InteractionHandler  # Start interaction.


PIDS = ["p001", "p002", "p003", "p004", "p005", "p006", "p007", "p008", "p009",
        "p010", "p011", "p012", "p013", "p014", "p015", "p016", "p017", "p018",
        "p019", "p020", "p021", "p022", "p023", "p024", "p025", "p026", "p027",
        "p028", "p029", "p030", "p031", "p032", "p101", "p102", "p103", "p104",
        "p105", "p201", "p202", "p203", "p204", "p205", "p301", "p302", "p303",
        "p304", "p305", "p306", "p307", "p999", "p998", "p997", "p206"]


BAG_PROCESS = None


def check_pid(pid):
    """ Check whether a given id exists in our list of acceptable ids. """
    return pid in PIDS


def check_session(session):
    """ Session numbers go from 1 to 10. """
    return session > 0 and session < 10


def check_name(name):
    """ Check whether a given name exists as an audio file in the audio files
    directory.
    """
    # If the name is not None, we should check whether it exists as an audio
    # file.
    if not name:
        print "Experimenter name is not valid!"
        return False

    # Read the audio files directory location from the toml config file.
    try:
        with open("config.toml") as tof:
            toml_data = toml.load(tof)
        # Directory of audio files.
        if "audio_base_dir" in toml_data:
            audio_base_dir = toml_data["audio_base_dir"]
            return os.path.isfile(audio_base_dir + name + ".wav")
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
    bagdir = "/home/robots/rr2_rosbags/"
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
        return subprocess.Popen(["rosbag", "record", "-a", "-O", bag],
                                shell=False)
    except Exception:
        print "Uh oh, something went wrong with rosbag recording!"
        raise


def close_rosbag(bagp):
    """ Close the rosbag and all its child processes. """
    print "Stopping rosbag recording if it's not stopped already..."
    process = psutil.Process(bagp.pid)
    for subp in process.get_children(recursive=True):
        subp.send_signal(signal.SIGINT)
    bagp.wait()


def signal_handler(sig, nal):
    """ Handle signals caught. """
    if sig == signal.SIGINT:
        print "Got keyboard interrupt! Exiting. {} {}".format(sig, nal)
        # Unsubscribe from stuff and cleanup before exiting.
        if BAG_PROCESS:
            close_rosbag(BAG_PROCESS)


def exit_nicely():
    """ Exit nicely. """
    print "Told to exit! Exiting..."
    if BAG_PROCESS:
        close_rosbag(BAG_PROCESS)


def launch(experimenter, participant, session, restart):
    """ Start the rosbag recording and launch the interaction. """
    # Set up signal handler to catch SIGINT (e.g., ctrl-c).
    signal.signal(signal.SIGINT, signal_handler)
    try:
        # Start the rosbag recording.
        global BAG_PROCESS
        BAG_PROCESS = start_rosbag(participant, session)
        # Start the interaction with the provided session number, participant
        # ID, entrainment set to true, experimenter name, and the restart
        # point (if any).
        InteractionHandler().launch_interaction(session, participant, True,
                                                experimenter, restart)

    except Exception as exc:
        print "Uh oh, something went wrong! {}".format(exc)
        if BAG_PROCESS:
            close_rosbag(BAG_PROCESS)
        raise

    # After the interaction ends, stop rosbag recording.
    if BAG_PROCESS:
        close_rosbag(BAG_PROCESS)
