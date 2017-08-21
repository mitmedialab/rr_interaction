#!/usr/bin/env python

# Jacqueline Kory Westlund
# August 2017
#
# The MIT License (MIT)
#
# Copyright (c) 2017 Personal Robots Group
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys # exit and argv
import json # For reading log config file.
import toml # For reading our config files.
import rospy # ROS
import argparse # To parse command line arguments.
import signal # catching SIGINT signal
import logging # log messages
import Queue # for getting messages from ROS callback threads
import datetime # for getting time deltas for timeouts
from rr_script_handler import rr_script_handler # plays back script lines
from rr_ros import rr_ros # we put all our ROS stuff here

class rr_interaction_node():
    """ The relational robot main interaction node orchestrates everything: what
    the robot should do, what is loaded on the tablet, what to do in response to
    stuff that happens on the tablet or from other sensors.

    This node sends ROS messages to an Opal device via a rosbridge_server
    websocket connection, and uses ROS to exchange messages with other relevant
    nodes (such as the audio entrainer and the Tega robot).
    """
    # Initialize the ROS node.
    # TODO If running on network where DNS does not resolve local
    # hostnames, get the public IP address of this machine and
    # export to the environment variable $ROS_IP to set the public
    # address of this node, so the user doesn't have to remember
    # to do this before starting the node.
    _ros_node = rospy.init_node('relational_robot', anonymous=True)
            # We could set the ROS log level here if we want:
            #log_level=rospy.DEBUG)
            # The rest of our logging is set up in the log config file.


    def __init__(self):
        """ Initialize anything that needs initialization """
        # Set up queue that we use to get messages from ROS callbacks.
        self._queue = Queue.Queue()
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        # Configure logging.
        try:
            config_file = "rr_log_config.json"
            with open(config_file) as json_file:
                json_data = json.load(json_file)
                logging.config.dictConfig(json_data)
                self._logger.debug("\n==============================\n" +
                    "STARTING\nLogger configuration:\n %s", json_data)
        except Exception as e:
            # Could not read config file -- use basic configuration.
            logging.basicConfig(filename="ss.log",
                    level=logging.DEBUG)
            self._logger.exception("ERROR! Could not read your json log "
                + "config file \"" + config_file + "\". Does the file "
                + "exist? Is it valid json?\n\nUsing default log setup to "
                + "log to \"ss.log\". Will not be logging to rosout!")


    def parse_arguments(self):
        # Parse python arguments.
        # The game node requires the session number and participant ID be
        # provided so the appropriate game scripts can be loaded.
        parser = argparse.ArgumentParser(
                formatter_class=argparse.RawDescriptionHelpFormatter,
                description='Start the SAR Social Stories game node, which '
                + 'orchestrates the game: loads scripts, uses ROS to tell the '
                + 'robot and tablet what to do.\nRequires roscore to be running'
                + ' and requires rosbridge_server for communication with the '
                + 'SAR opal tablet (where game content is shown).')
        parser.add_argument('session', action='store',
               nargs='?', type=int, default=-1, help='Indicate which session'
               + ' this is so the appropriate game scripts can be loaded.')
        parser.add_argument('participant',
               action='store', nargs='?', type=str, default='DEMO', help=
               'Indicate which participant this is so the appropriate game '
               + 'scripts can be loaded.')

        # Parse the args we got, and print them out.
        args = parser.parse_args()
        self._logger.debug("Args received: %s", args)

        # Return the session number and participant ID so they can be
        # used by the game launcher, where they will be used to load
        # appropriate game scripts.
        #
        # If the session number doesn't make sense, throw an error.
        if args.session < -1:
            raise ValueError("Session number out of range. Should be -1 to "
                 "play the demo or a positive integer to play a particular "
                 "session.")

        # If the args indicate that this is a demo, return demo args.
        if args.session <= 0 or args.participant.lower() == "demo":
            return (-1, "DEMO")

        # Otherwise, return the provided session and ID.
        else:
            return (args.session, args.participant)


    def launch_game(self, session, participant):
        """ Load game based on the current session and participant """
        # Log session and participant ID.
        self._logger.info("\n==============================\n" + "RELATIONAL "
                "ROBOT \nSession: %s, Participant ID: %s", session, participant)

        # Set up ROS node publishers and subscribers.
        self._ros_ss = rr_ros(self._queue)

        # Read config file to get relative file path to interaction scripts.
        try:
            config_file = "../config.demo.toml" if participant == "DEMO" \
                    else "../config.toml"
            with open(config_file) as tf:
                toml_data = toml.load(tf)
                self._logger.debug("Reading game config file...: %s", toml_data)
                # Directory with scripts for this study.
                if "study_path" in toml_data:
                    study_path = toml_data["study_path"]
                else:
                    self._logger.error("Could not read relative path to game "
                        + "scripts! Expected option \"study_path\" to be in "
                        + "the config file. Exiting because we need the "
                        + "scripts to run the game.")
                    return
                # Study script config file location.
                if "study_config" in toml_data:
                    study_config = toml_data["study_config"]
                else:
                    self._logger.error("Could not read name of study_config!"
                            + "Expected option \"study_config\" to be in config"
                            + " file. Exiting because we need the study config"
                            + " to continue.")
                    return
                # Directory of story scripts.
                if "story_script_path" in toml_data:
                    story_script_path = toml_data["story_script_path"]
                else:
                    self._logger.error("Could not read path to story scripts! "
                        + "Expected option \"story_script_path\" to be in "
                        + "config file. Assuming story scripts are in the main "
                        + "study directory and not a sub-directory.")
                    story_script_path = None
                # Directory of session scripts.
                if "session_script_path" in toml_data:
                    session_script_path = toml_data["session_script_path"]
                else:
                    self._logger.error("Could not read path to session scripts! "
                        + "Expected option \"session_script_path\" to be in "
                        + "config file. Assuming session scripts are in the main"
                        + " study directory and not a sub-directory.")
                    session_script_path = None
                # Directory of audio files.
                if "audio_base_dir" in toml_data:
                    audio_base_dir = toml_data["audio_base_dir"]
                else:
                    self._logger.error("Could not read audio base directory "
                            + "path! Expected option \"audio_base_dir\" to be"
                            + "in config file. Assuming audio files are in the"
                            + " main study directory.")
                    audio_base_dir = None
                # Directory of viseme files.
                if "viseme_base_dir" in toml_data:
                    viseme_base_dir = toml_data["viseme_base_dir"]
                else:
                    self._logger.error("Could not read viseme base directory "
                            + "path! Expected option \"viseme_base_dir\" to be"
                            + "in config file.Assuming audio files are in the"
                            + " main study directory.")
                    viseme_base_dir = None
        except Exception as e:
            self._logger.exception("Could not read your toml config file \""
                + config_file + "\". Does the file exist? Is it valid toml?"
                + " Exiting because we need the config file to continue.")
            return

        # Load script. #TODO update args.
        try:
            script_handler = rr_script_handler(self._ros_ss, session,
                participant, study_path, story_script_path,
                session_script_path, database, self._queue,
                percent_correct_to_level)
        except IOError as e:
            self._logger.exception("Did not load the session script... exiting "
                + "because we need the session script to continue.")
            return
        else:
            # Flag to indicate whether we should exit.
            self._stop = False

            # Flags for game control.
            started = False
            paused = False
            log_timer = datetime.datetime.now()

            # Set up signal handler to catch SIGINT (e.g., ctrl-c).
            signal.signal(signal.SIGINT, self._signal_handler)

            # Ready to start the game. Send a "READY" message.
            self._logger.info("Ready to start!")
            self._ros_ss.send_game_state("READY")

            while (not self._stop):
                try:
                    try:
                        # Get data from queue if any is there, but don't
                        # wait if there isn't.
                        msg = self._queue.get(False)
                    except Queue.Empty:
                        # no data yet!
                        pass
                    else:
                        # Got a message! Parse:
                        # Wait for START command before starting to
                        # iterate over the script.
                        if "START" in msg and not started:
                            self._logger.info("Starting game!")
                            # Pass on the start level, if it was given.
                            msg_parts = msg.split("\t")
                            if len(msg_parts) > 1:
                                try:
                                    script_handler.set_start_level(
                                        int(msg_parts[1]))
                                    self._logger.info("Got start level: "
                                        + msg_parts[1])
                                except ValueError:
                                    self._logger.warning("Was given a start " +
                                        "level that wasn't an int! "
                                        + msg_parts[1])
                            started = True
                            # Announce the game is starting.
                            self._ros_ss.send_game_state("START")
                            self._ros_ss.send_game_state("IN_PROGRESS")

                        # If we get a PAUSE command, pause iteration over
                        # the script.
                        elif "PAUSE" in msg and not paused:
                            self._logger.info("Game paused!")
                            log_timer = datetime.datetime.now()
                            paused = True
                            script_handler.pause_game_timer()
                            # Announce the game is pausing.
                            self._ros_ss.send_game_state("PAUSE")

                        # If we are paused and get a CONTINUE command,
                        # we can resume iterating over the script. If
                        # we're not paused, ignore.
                        elif "CONTINUE" in msg and paused:
                            self._logger.info("Resuming game!")
                            paused = False
                            script_handler.resume_game_timer()
                            # Announce the game is resuming.
                            self._ros_ss.send_game_state("IN_PROGRESS")

                        # When we receive an END command, we need to
                        # exit gracefully. Stop all repeating scripts
                        # and story scripts, go directly to the end.
                        elif "END" in msg and started:
                            self._logger.info("Ending game!")
                            script_handler.set_end_game()

                        # When we receive a WAIT_FOR_RESPONSE command,
                        # we can unpause the game, but go directly to
                        # waiting for a user response rather than
                        # reading the next script line.
                        elif "WAIT_FOR_RESPONSE" in msg and started:
                            self._logger.info("Waiting for user response!")
                            if (script_handler. \
                                wait_for_last_response_again()):
                                # If we get a response, we can unpause
                                # (but we may not have been paused).
                                paused = False
                                script_handler.resume_game_timer()
                                # Announce the game is resuming.
                                self._ros_ss.send_game_state("IN_PROGRESS")
                            else:
                                # We timed out again, don't resume.
                                self._logger.info("Did not get response!")

                        # When we receive a SKIP_RESPONSE command, we
                        # unpause the game, and instead of waiting for
                        # user response, we skip waiting, and continue
                        # with the next script line.
                        elif "SKIP_RESPONSE" in msg and started:
                            self._logger.info("Skipping waiting for user " +
                                "response!")
                            # Treat the skipped response as a NO or as
                            # INCORRECT, then let the game resume play
                            # normally.
                            script_handler.skip_wait_for_response()
                            # Unpause and continue the game.
                            paused = False
                            script_handler.resume_game_timer()
                            # Announce the game is resuming.
                            self._ros_ss.send_game_state("IN_PROGRESS")

                    # If the game has been started and is not paused,
                    # parse and handle the next script line.
                    if started and not paused:
                        script_handler.iterate_once()

                    elif not started or paused:
                        # Print a log message periodically stating that
                        # we are waiting for a command to continue.
                        if (datetime.datetime.now() - log_timer > \
                                datetime.timedelta(seconds=int(5))):
                            if paused:
                                self._logger.info("Game paused... waiting for "
                                + "command to continue or skip response.")
                            elif not started:
                                self._logger.info("Waiting for command to "
                                    + "start.")
                            log_timer = datetime.datetime.now()

                except StopIteration as e:
                    self._logger.info("Finished script!")
                    # Send message to announce the game is over.
                    if "performance" in dir(e):
                        self._ros_ss.send_game_state("END", e.performance)
                    else:
                        self._ros_ss.send_game_state("END")
                    break

            # TODO wait after exiting this loop for the main
            # SessionManager to close the process??


    def _signal_handler(self, sig, frame):
        """ Handle signals caught """
        if sig == signal.SIGINT:
            self._logger.info("Got keyboard interrupt! Exiting.")
            self._stop = True
            exit("Interrupted by user.")


if __name__ == '__main__':
    # Try launching the game!
    try:
        main_node = rr_interaction_node()
        (session, participant) = main_node.parse_arguments()
        main_node.launch_game(session, participant)

    # If roscore isn't running or shuts down unexpectedly...
    except rospy.ROSInterruptException:
        self._logger.exception('ROS node shutdown')
        pass

