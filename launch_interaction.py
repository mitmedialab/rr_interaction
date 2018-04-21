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

import argparse  # For getting optional restart-related arguments.
import signal  # Catching SIGINT.
import interaction_launcher


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
        if interaction_launcher.check_name(name):
            print "\tOkay! Hi {}, have fun running the experiment!".format(
                    (name[:1].upper() + name[1:]))
            break
        else:
            print "{} is not a valid experimenter name. Try again. Example: " \
                  "jackie".format(name)

    # 2. The participant ID (used in the rosbag filename and to start the
    #    interaction with the correct setup). Acceptable IDs will follow the
    #    pattern "p000", "p001", etc.
    while True:
        pid = raw_input("Enter participant ID: ")
        if interaction_launcher.check_pid(pid):
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
            if interaction_launcher.check_session(session):
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


def signal_handler(sig, nal):
    """ Handle signals caught. """
    if sig == signal.SIGINT:
        print "Got keyboard interrupt! Exiting. {} {}".format(sig, nal)
        # Unsubscribe from stuff and cleanup before exiting.
        interaction_launcher.exit_nicely()


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
    # Set up signal handler to catch SIGINT (e.g., ctrl-c).
    signal.signal(signal.SIGINT, signal_handler)

    PARSER = argparse.ArgumentParser(
        description="Run the interaction. Optionally, restart at a later point"
                    "in the script.")
    PARSER.add_argument(
        "-r", "--restart", choices=["beginning", "intro", "apt", "sdt", "rs1",
                                    "rs2", "cs1", "pc", "photo", "close"],
        type=str, dest="restart", help="Restart interaction at"
        "a designated restart point: intro, apt (anomalous picture task), sdt "
        "(self-disclosure task), rs1 (robot's first story), rs2 (robot's "
        "second story), photo (take photo with robot), close (the"
        " interaction closing).")
    PARSER.add_argument(
        "-e", "--experimenter", type=str, dest="experimenter",
        help="Experimenter name.")
    PARSER.add_argument(
        "-p", "--participant", type=str, dest="participant",
        help="Participant ID.")
    PARSER.add_argument(
        "-s", "--session", type=int, dest="session", help="Session name.")
    PARSER.add_argument(
        "-n", "--non-interactive", action="store_true",
        dest="non_interactive", help="Don't ask questions.")
    ARGS = PARSER.parse_args()
    if ARGS.restart:
        RESTART = ARGS.restart
    else:
        RESTART = None

    # If we have valid args, don't ask for them.
    if interaction_launcher.check_name(ARGS.experimenter) \
            and interaction_launcher.check_pid(ARGS.participant) \
            and interaction_launcher.check_session(ARGS.session):
        interaction_launcher.launch(
            ARGS.experimenter, ARGS.participant, ARGS.session, RESTART)
    elif ARGS.non_interactive:
        exit(1)
    else:
        interaction_launcher.launch(*get_args(), restart=RESTART)

    # If we get here we should try to close the interaction nicely.
    interaction_launcher.exit_nicely()
