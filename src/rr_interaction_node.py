#!/usr/bin/env python
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

import json  # For reading log config file.
import toml  # For reading our config files.
import rospy  # ROS
import argparse  # To parse command line arguments.
import signal  # catching SIGINT signal
import logging  # log messages
import Queue  # for getting messages from ROS callback threads
import datetime  # for getting time deltas for timeouts
from rr_script_handler import ScriptHandler  # plays back script lines
from rr_ros import RosNode  # we put all our ROS stuff here


class InteractionHandler(object):
    """ The relational robot main interaction node orchestrates everything:
    what the robot should do, what is loaded on the tablet, what to do in
    response to stuff that happens on the tablet or from other sensors.

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
    # log_level=rospy.DEBUG)
    # The rest of our logging is set up in the log config file.

    def __init__(self):
        """ Initialize anything that needs initialization. """
        # Create variable for our ROS node. We give it a value later.
        self._ros_ss = None
        # Flag to indicate whether we should exit. We don't start stopped.
        self._stop = False
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
                                   "STARTING\nLogger configuration:\n %s",
                                   json_data)
        except Exception as exc:  # pylint: disable=broad-except
            # Could not read config file -- use basic configuration.
            print "Error loading log config: {}".format(exc)
            logging.basicConfig(filename="ss.log", level=logging.DEBUG)
            self._logger.exception("""ERROR! Could not read your json log
                                   config file \"" + config_file + "\". Does
                                   the file exist? Is it valid json?\n\nUsing
                                   default log setup to log to \"ss.log\". Will
                                   not be logging to rosout!""")

    def parse_arguments(self):
        """ Parse python arguments. The interaction handler requires the
        session number and participant ID be provided so the appropriate
        scripts and performance logs can be loaded.
        """
        parser = argparse.ArgumentParser(description="""Start the main
            interaction handler, which orchestrates the interaction: loads
            scripts, uses ROS to tell the robot and tablet what to do.
            \nRequires roscore to be running and requires rosbridge_server
            for communication with the Opal tablet (where some interaction
            content is shown).""")
        parser.add_argument("session", action="store", nargs="?", type=int,
                            default=-1, help="Indicate which session this is"
                            "so the appropriate scripts can be loaded."
                            "If not specified, defaults to the demo.")
        parser.add_argument("participant", action="store", nargs="?", type=str,
                            default="DEMO", help="Indicate which participant"
                            "this is so the appropriate scripts can be loaded."
                            "If not specified, defaults to the demo.")
        # The user can decide to send audio directly to the robot or to go
        # through the audio entrainment module first.
        parser.add_argument("-e", "--use-entrainer", action='store_true',
                            default=False, dest="use_entrainer", help="""Send
                            audio to the entrainer on the way to the robot.""")
        # The user can specify a participant-specific toml configuration file
        # if the interaction session should be personalized to individuals.
        parser.add_argument("-p, --pconfig", type=str, nargs="?", default=None,
                            action="store", dest="participant_config",
                            help="TOML participant-specific config file. "
                            "Needed to personalize the interaction to "
                            "individuals. If not specified, defaults to the "
                            "demo.")
        # The user can specify the current experimenter so that if a script
        # contains the keyword "<experimenter_name>", it can be replaced with
        # the name of the current experimenter.
        parser.add_argument("-x", "--experimenter", action="store", nargs="?",
                            type=str, default="", help="Provide the name of "
                            "the person running this interaction session (the "
                            "experimenter) so they can be referred to by name")

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
                             "play the demo or a positive integer to play a "
                             "particular session.")

        # If the args indicate that this is a demo, return demo args.
        if args.session <= 0 or args.participant.lower() == "demo" or \
                not args.participant_config:
            return (-1, "DEMO", args.use_entrainer, None, None)

        # Otherwise, return the provided session and ID.
        else:
            return (args.session, args.participant, args.use_entrainer,
                    args.participant_config, args.experimenter.lower())

    def _read_config(self, config):
        """ Read in the main toml config file. """
        # We don't actually have too many branches here; we need to try reading
        # each value from the config file separately.
        # pylint: disable=too-many-branches
        try:
            with open(config) as tof:
                toml_data = toml.load(tof)
            self._logger.debug("Reading config file...: {}".format(toml_data))
            # Directory with scripts for this study.
            if "study_path" in toml_data:
                study_path = toml_data["study_path"]
            else:
                self._logger.error("Could not read path to interaction "
                                   "scripts! Expected option \"study_path\" to"
                                   "be in the config file. Exiting because we "
                                   "need the scripts to run the interaction.")
                exit(1)
            # Study script config file location.
            if "script_config" in toml_data:
                script_config = toml_data["script_config"]
            else:
                self._logger.error("Could not read name of script_config! "
                                   "Expected option \"script_config\" to be"
                                   " in config file. Exiting because we "
                                   "need the study config to continue.")
                exit(1)
            # Directory of story scripts.
            if "story_script_path" in toml_data:
                story_script_path = toml_data["story_script_path"]
            else:
                self._logger.error("Could not read path to story scripts! "
                                   "Expected option \"story_script_path\" to "
                                   "be in config file. Assuming story scripts "
                                   "are in the main study directory and not a "
                                   "sub-directory.")
                story_script_path = None
            # Directory of session scripts.
            if "session_script_path" in toml_data:
                session_script_path = toml_data["session_script_path"]
            else:
                self._logger.error("Could not read path to session scripts! "
                                   "Expected option \"session_script_path\" to"
                                   " be in config file. Assuming session "
                                   "scripts are in the main study directory "
                                   "and not a sub-directory.")
                session_script_path = None
            # Directory of audio files.
            if "audio_base_dir" in toml_data:
                audio_base_dir = toml_data["audio_base_dir"]
            else:
                self._logger.error("Could not read audio base directory path! "
                                   "Expected option \"audio_base_dir\" to be "
                                   "in config file. Assuming audio files are "
                                   "in the main study directory.")
                audio_base_dir = None
            # Directory of viseme files.
            if "viseme_base_dir" in toml_data:
                viseme_base_dir = toml_data["viseme_base_dir"]
            else:
                self._logger.error("Could not read viseme base directory path!"
                                   " Expected option \"viseme_base_dir\" to be"
                                   " in config file.Assuming audio files are "
                                   "in the main study directory.")
                viseme_base_dir = None
            # Directory where any output should be saved.
            if "output_dir" in toml_data:
                output_dir = toml_data["output_dir"]
            else:
                self._logger.error("Could not read path to the output  "
                                   "directory! Expected option \"output_dir\""
                                   "to be in the config file. Defaulting to "
                                   "saving in the current working directory.")
                output_dir = ""
        except Exception as exc:  # pylint: disable=broad-except
            self._logger.exception("Could not read your toml config file \"" +
                                   str(config) + "\". Does the file exist? Is "
                                   "it valid toml? Exiting because we need the"
                                   " config file to continue. {}".format(exc))
            exit(1)
        return study_path, script_config, story_script_path, \
            session_script_path, audio_base_dir, viseme_base_dir, output_dir

    def launch_interaction(self, session, participant, entrain, pconfig,
                           experimenter):
        """ Launch interaction based on the current session and participant.
        """
        # Log session and participant ID.
        self._logger.info("\n==============================\nRELATIONAL ROBOT"
                          "\nSession: {}, Participant ID: {}".format(
                              session, participant))

        # Set up ROS node publishers and subscribers.
        self._ros_ss = RosNode(self._queue)

        # Read config file to get paths to interaction scripts, script
        # directories, and more. If this is a demo interaction, load the demo
        # config file; otherwise try reading in the regular config file.
        study_path, script_config, story_script_path, session_script_path, \
            audio_base_dir, viseme_base_dir, output_dir = self._read_config(
                "config.demo.toml" if participant == "DEMO" else
                "config.toml")

        # Load script.
        try:
            script_handler = ScriptHandler(self._ros_ss, session, participant,
                                           study_path, story_script_path,
                                           session_script_path, script_config,
                                           pconfig, audio_base_dir,
                                           viseme_base_dir, output_dir,
                                           entrain, experimenter)
        except IOError as ioe:
            self._logger.exception("Did not load the session script... exiting"
                                   " because we need the session script to "
                                   "continue. Error: {}".format(ioe))
            exit(1)

        # We've loaded a script and are all configured. Start!
        self.run_interaction(script_handler)

    def run_interaction(self, script_handler):
        """ Run the interaction until we reach the end of the script or are
        told to exit.
        """
        # Flags for interaction control.
        paused = False
        log_timer = datetime.datetime.now()

        # Set up signal handler to catch SIGINT (e.g., ctrl-c).
        signal.signal(signal.SIGINT, self._signal_handler)

        # Start the interaction!
        self._logger.info("Starting interaction!")
        # Loop until we reach the end of the script or are told to exit.
        # TODO We should receive PAUSE, RESUME, and EXIT messages in case we
        # need to pause partway through the interaction or if we need to exit
        # early.
        while not self._stop:
            try:
                try:
                    # Get data from queue if any is there, but don't
                    # wait if there isn't.
                    msg = self._queue.get(False)
                except Queue.Empty:
                    # No data yet!
                    pass
                else:
                    # Got a message! Parse:
                    # If we get a PAUSE command, pause script iteration.
                    if "PAUSE" in msg and not paused:
                        self._logger.info("Interaction paused!")
                        log_timer = datetime.datetime.now()
                        paused = True
                        script_handler.pause_interaction_timer()

                    # If we are paused and get a RESUME command, we can resume
                    # iterating over the script. If we're not paused, ignore.
                    elif "RESUME" in msg and paused:
                        self._logger.info("Resuming interaction!")
                        paused = False
                        script_handler.resume_interaction_timer()

                    # When we receive an EXIT command, we need to exit
                    # gracefully. Stop all repeating scripts and story scripts,
                    # go directly to the end.
                    elif "EXIT" in msg:
                        self._logger.info("Ending interaction!")
                        script_handler.set_end_interaction()

                # If the interaction is not paused, parse and handle the next
                # script line.
                if not paused:
                    script_handler.iterate_once()

                # If the interaction is paused, print a periodic log message
                # stating that we are waiting for a RESUME command.
                elif (datetime.datetime.now() - log_timer >
                        datetime.timedelta(seconds=int(5))):
                    self._logger.info("Interaction paused... waiting for "
                                      "command to resume.")
                    log_timer = datetime.datetime.now()

            except StopIteration:
                self._logger.info("Finished script!")
                break

    def _signal_handler(self, sig, nal):
        """ Handle signals caught. """
        if sig == signal.SIGINT:
            self._logger.info("Got keyboard interrupt! Exiting.")
            exit("Interrupted by user.")


if __name__ == '__main__':
    # Try launching the interaction!
    try:
        INTERACTION_HANDLER = InteractionHandler()
        (SESSION, PARTICIPANT, ENTRAIN, PCONFIG, EXPERIMENTER) = \
            INTERACTION_HANDLER.parse_arguments()
        INTERACTION_HANDLER.launch_interaction(SESSION, PARTICIPANT, ENTRAIN,
                                               PCONFIG, EXPERIMENTER)

    # If roscore isn't running or shuts down unexpectedly...
    except rospy.ROSInterruptException:
        print "ROS node shutdown"
