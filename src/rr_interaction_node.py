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
import rospy  # ROS
import argparse  # To parse command line arguments.
import signal  # catching SIGINT signal
import logging  # log messages
import os  # For determining if files exist.
import toml  # For reading the study config file.
import Queue  # for getting messages from ROS callback threads
import datetime  # for getting time deltas for timeouts
from rr_script_handler import ScriptHandler  # plays back script lines
from rr_ros import RosNode  # we put all our ROS stuff here
from rr_msgs.msg import UserInput  # For user input form response constants.


class InteractionHandler(object):
    """ The relational robot main interaction node orchestrates everything:
    what the robot should do, what is loaded on the tablet, what to do in
    response to stuff that happens on the tablet or from other sensors.

    This node sends ROS messages to an Opal device via a rosbridge_server
    websocket connection, and uses ROS to exchange messages with other relevant
    nodes (such as the audio entrainer and the Tega robot).
    """
    # If you *import* this module somewhere else, the code where you import it
    # will hang unless rospy.init_node() is called from within a function. If
    # rospy.init_node() is called outside a function, as a module-level thing,
    # the code where you import this module will hang. It's weird, yes; there
    # is one ros answer that talks about it a little: https://answers.ros.org/
    # question/266612/rospy-init_node-inside-imported-file/
    # Also!
    # If you do this - putting the init_node() call in a function - then a lot
    # of the log output is no longer printed to the terminal screen or put in
    # the rr_debug log file. Not sure why. Instead, you'll find that output in
    # the ROS log output, which you can find in "~/.ros/log/" or
    # "$ROS_ROOT/log". You can then tail the appropriate file and see the log
    # output as it happens, e.g.,: "tail -f ~/.ros/log/relational_robot.log".
    # _ros_node = rospy.init_node('relational_robot', anonymous=False)
    def init_ros(self):
        """ Initialize the ROS node. """
        self._ros_node = rospy.init_node('relational_robot', anonymous=False)
        # We could set the ROS log level here if we want:
        # log_level=rospy.DEBUG)
        # The rest of our logging is set up in the log config file.

    def __init__(self):
        """ Initialize anything that needs initialization. """
        # Our ROS node. We give it a value later.
        self._ros_node = None
        # The class that handles all ROS communication for us.
        self._ros_handler = None
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
            self._logger.error("Error loading log config: {}".format(exc))
            logging.basicConfig(filename="rr.log", level=logging.DEBUG)
            self._logger.exception("""ERROR! Could not read your json log
                                   config file \"" + config_file + "\". Does
                                   the file exist? Is it valid json?\n\nUsing
                                   default log setup to log to \"rr.log\". Will
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
            return (-1, "DEMO", args.use_entrainer, None)

        # Otherwise, return the provided session and ID.
        else:
            return (args.session, args.participant, args.use_entrainer,
                    args.experimenter.lower())

    def _read_main_config(self, config):
        """ Read in the main toml config file. """
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
                story_script_path = ""
            # Directory of session scripts.
            if "session_script_path" in toml_data:
                session_script_path = toml_data["session_script_path"]
            else:
                self._logger.error("Could not read path to session scripts! "
                                   "Expected option \"session_script_path\" to"
                                   " be in config file. Assuming session "
                                   "scripts are in the main study directory "
                                   "and not a sub-directory.")
                session_script_path = ""
            # Directory of audio files.
            if "audio_base_dir" in toml_data:
                audio_base_dir = toml_data["audio_base_dir"]
            else:
                self._logger.error("Could not read audio base directory path! "
                                   "Expected option \"audio_base_dir\" to be "
                                   "in config file. Assuming audio files are "
                                   "in the main study directory.")
                audio_base_dir = ""
            # Directory of viseme files.
            if "viseme_base_dir" in toml_data:
                viseme_base_dir = toml_data["viseme_base_dir"]
            else:
                self._logger.error("Could not read viseme base directory path!"
                                   " Expected option \"viseme_base_dir\" to be"
                                   " in config file.Assuming audio files are "
                                   "in the main study directory.")
                viseme_base_dir = ""
            # Directory where any output should be saved.
            if "output_dir" in toml_data:
                output_dir = toml_data["output_dir"]
            else:
                self._logger.error("Could not read path to the output  "
                                   "directory! Expected option \"output_dir\""
                                   "to be in the config file. Defaulting to "
                                   "saving in the current working directory.")
                output_dir = ""
            # Directory where participant config files are located.
            if "pconfig_dir" in toml_data:
                pconfig_dir = toml_data["pconfig_dir"]
            else:
                self._logger.error("Could not read path to the participant "
                                   "config directory! Expected \"pconfig_dir\""
                                   "to be in the config file. Defaulting to "
                                   "checking the current working directory.")
                pconfig_dir = ""
            # Naming pattern for participant config files.
            if "pconfig_name" in toml_data:
                pconfig_name = toml_data["pconfig_name"]
            else:
                self._logger.error("Could not read name for the participant "
                                   "config files! Expected \"pconfig_name\""
                                   "to be in the config file. Defaulting to "
                                   "\"participant_config\".")
                pconfig_name = "participant_config"
        except Exception as exc:  # pylint: disable=broad-except
            self._logger.exception("Could not read your toml config file \"" +
                                   str(config) + "\". Does the file exist? Is "
                                   "it valid toml? Exiting because we need the"
                                   " config file to continue. {}".format(exc))
            exit(1)
        return study_path, script_config, story_script_path, \
            session_script_path, audio_base_dir, viseme_base_dir, output_dir, \
            pconfig_dir, pconfig_name

    def _load_toml_config(self, config):
        """ Load in a toml config file for later reference. """
        try:
            with open(config) as tof:
                toml_data = toml.load(tof)
            # self._logger.debug("Reading toml config...: {}".format(
            #    toml_data))
            return toml_data
        except Exception as exc:  # pylint: disable=broad-except
            self._logger.exception("""Could not read your toml config file
                                   \"{}\". Does the file exist? Is it valid
                                   toml? Exiting because we need the config
                                   file to continue. {}""".format(config, exc))
            exit(1)

    def _get_participant_config(self, pconfig_dir, pconfig_name):
        """ Given the directory with the participant config files in it, load
        the current one (i.e. the one with the highest number in its filename).
        """
        # For all the files in the provided directory, check whether they
        # follow the participant config file naming schema. Then sort them so
        # the highest numbered file is at the end, since that's the one we
        # want.
        if pconfig_dir == "":
            pconfig_dir = os.getcwd()
        confs = sorted([c for c in os.listdir(pconfig_dir) if
                       (c.startswith(pconfig_name) and
                        c.endswith(".toml"))])
        if confs:
            self._logger.info("Found participant config: {}".format(confs[-1]))
            return pconfig_dir + confs[-1]
        else:
            return None

    def launch_interaction(self, session, participant, use_entrainer,
                           experimenter):
        """ Launch interaction based on the current session and participant.
        """
        # Log session and participant ID.
        self._logger.info("\n==============================\nRELATIONAL ROBOT"
                          "\nSession: {}, Participant ID: {}".format(
                              session, participant))

        # Read the main config file to get paths to interaction scripts, script
        # directories, and more. If this is a demo interaction, load the demo
        # config file; otherwise try reading in the regular config file.
        # Save study, script, audio, and viseme paths so we can load scripts
        # and audio later.
        study_path, script_config_path, story_script_path, \
            session_script_path, audio_base_dir, viseme_base_dir, output_dir, \
            pconfig_dir, pconfig_name = self._read_main_config(
                "config.demo.toml" if participant == "DEMO" else "config.toml")

        # Get the script config, if this is not the demo.
        if "DEMO" not in participant:
            script_config = self._load_toml_config(script_config_path)
        else:
            script_config = {}

        # Read in the main participant config file to get this participant's
        # configuration.
        if "DEMO" in participant:
            pconfig = {}
        else:
            pconfig = self._load_toml_config(self._get_participant_config(
                pconfig_dir, pconfig_name))
            self._logger.debug(pconfig)
            if participant not in pconfig:
                self._logger.warn("{} is not in the participant config file "
                                  "we loaded! We can't personalize!".format(
                                       participant))
            elif str(session) not in pconfig[participant]:
                self._logger.warn("Session {} is not in the participant config"
                                  " file we loaded for {}! We can't "
                                  "personalize!".format(session, participant))
            else:
                # Since we are only running one participant and one session, we
                # only care about the configuration for this participant.
                pconfig = pconfig[participant]

        # For the RR2 study, if the participant's condition is set to NR,
        # then set entrain=False so that audio is streamed through the
        # entrainer but is not actually entrained.
        entrain = True
        backchannel_random = False
        if "condition" in pconfig:
            if "NR" in pconfig["condition"]:
                entrain = False
                backchannel_random = True

        # Set up ROS node publishers and subscribers. Pass in the lists of
        # backchannel actions from the script config file.
        if "backchannel_actions" in script_config:
            backchannel_actions = script_config["backchannel_actions"]
        else:
            backchannel_actions = {}

        self.init_ros()
        self._ros_handler = RosNode(self._queue, use_entrainer, entrain,
                                    audio_base_dir, viseme_base_dir,
                                    backchannel_actions, backchannel_random)

        # Load the session script. The script handler loads the main config
        # file since that file mostly has directories to where scripts and
        # other files are, which we don't need to know here but it does.
        try:
            script_handler = ScriptHandler(self._ros_handler, session,
                                           participant, experimenter,
                                           study_path, script_config,
                                           story_script_path,
                                           session_script_path,
                                           output_dir, pconfig)
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
        started = False
        log_timer = datetime.datetime.now()

        # Set up signal handler to catch SIGINT (e.g., ctrl-c).
        signal.signal(signal.SIGINT, self._signal_handler)

        # Start the interaction!
        self._logger.info("Starting interaction!")
        # Loop until we reach the end of the script or are told to exit.
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
                    # If we get a START message, start the interaction.
                    if UserInput.START in msg and not paused:
                        self._logger.info("Starting interaction!")
                        self._ros_handler.send_interaction_state(
                                state="start interaction")
                        started = True

                    # If we get a PAUSE command, pause script iteration.
                    if UserInput.PAUSE in msg and not paused:
                        self._logger.info("Interaction paused!")
                        self._ros_handler.send_interaction_state(
                                state="pause interaction")
                        log_timer = datetime.datetime.now()
                        paused = True
                        script_handler.pause_interaction_timer()

                    # If we are paused and get a RESUME command, we can resume
                    # iterating over the script. If we're not paused, ignore.
                    elif UserInput.RESUME in msg and paused:
                        self._logger.info("Resuming interaction!")
                        self._ros_handler.send_interaction_state(
                                state="resume interaction")
                        paused = False
                        script_handler.resume_interaction_timer()

                    # When we receive an STOP command, we need to exit
                    # gracefully. Stop all repeating scripts and story scripts,
                    # go directly to the end.
                    elif UserInput.STOP in msg:
                        self._logger.info("Ending interaction!")
                        script_handler.set_end_interaction()

                # If the interaction has started and is not paused, parse and
                # handle the next script line.
                if started and not paused:
                    script_handler.iterate_once()

                # If the interaction has not started yet or is paused, print a
                # periodic log message stating that we are waiting for a START
                # command or a RESUME command.
                elif (not started or paused) and \
                        (datetime.datetime.now() - log_timer >
                            datetime.timedelta(seconds=int(5))):
                    if paused:
                        self._logger.info("Interaction paused... waiting "
                                          "for command to resume.")
                    elif not started:
                        self._logger.info("Waiting for command to start.")
                    log_timer = datetime.datetime.now()

            except StopIteration:
                self._logger.info("Finished script!")
                self._ros_handler.send_interaction_state(
                        state="end interaction finished script")
                break

    def _signal_handler(self, sig, nal):
        """ Handle signals caught. """
        if sig == signal.SIGINT:
            self._logger.info("Got keyboard interrupt! Exiting. {} {}".format(
                sig, nal))
            self._ros_handler.send_interaction_state(
                    state="end interaction by user")
            exit("Interrupted by user.")


if __name__ == '__main__':
    # Try launching the interaction!
    try:
        INTERACTION_HANDLER = InteractionHandler()
        (SESSION, PARTICIPANT, ENTRAIN, EXPERIMENTER) = \
            INTERACTION_HANDLER.parse_arguments()
        INTERACTION_HANDLER.launch_interaction(SESSION, PARTICIPANT, ENTRAIN,
                                               EXPERIMENTER)

    # If roscore isn't running or shuts down unexpectedly...
    except rospy.ROSInterruptException:
        print "ROS node shutdown"
