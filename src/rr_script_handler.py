# pylint: disable=too-many-lines
# The module has a fine number of lines, thanks pylint, since half of them are
# comments anyway.
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

import datetime  # For getting time deltas for timeouts
import time  # For sleep
import json  # For packing ros message properties
import toml  # For reading the study config file.
import random  # For picking robot responses and shuffling answer options
import logging  # Log messages
from rr_errors import NoStoryFound  # Custom exception when no stories found
from rr_script_parser import ScriptParser  # Parses scripts
from rr_performance_logger import PerformanceLogger  # Logs participant data
from asr_google_cloud.msg import AsrCommand  # Tell ASR to start/stop.
from r1d1_msgs.msg import TegaAction  # Tell Tega to do different fidgets.


class ScriptHandler(object):
    """ The script handler parses and deals with script lines. It uses the
    script parser to get the next line in a script. We keep loading script
    lines and parsing script lines separate on the offchance that we might want
    to replace how scripts are stored and accessed (e.g., in a database versus
    in text files).
    """
    # We use all our instance attributes to track many aspects of the robot
    # interaction, so it's not actually too many.
    # pylint: disable=too-many-instance-attributes

    # Constants for script playback:
    # Time to pause after showing answer feedback and playing robot
    # feedback speech before moving on to the next question.
    ANSWER_FEEDBACK_PAUSE_TIME = 2
    # Time to wait for robot to finish speaking or acting before moving on to
    # the next script line (in seconds). This is a big number because the
    # robot might have to say something long at some point.
    WAIT_TIME = 30

    def __init__(self, ros_node, session, participant, study_path,
                 story_script_path, session_script_path, script_config,
                 p_config, audio_base_dir, viseme_base_dir, output_dir,
                 entrain):
        """ Save references to ROS connection and logger, get scripts and
        set up to read script lines.
        """
        # Sorry pylint, in this case, we need all the arguments.
        # pylint: disable=too-many-arguments
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up script handler...")

        # Get the study config file.
        self._script_config = self._load_toml_config(script_config)

        # Do we send audio to the audio entrainer on the way to the robot?
        self._use_entrainer = entrain

        # Save reference to our ros node so we can publish messages.
        self._ros_node = ros_node

        # Save study, script, audio, and viseme paths so we can load scripts
        # and audio later.
        self._audio_base_dir = audio_base_dir
        self._viseme_base_dir = viseme_base_dir
        self._study_path = study_path

        if story_script_path is None:
            self._story_script_path = ""
        else:
            self._story_script_path = story_script_path

        if session_script_path is None:
            self._session_script_path = ""
        else:
            self._session_script_path = session_script_path

        # Load the participant configuration file so we can look up the
        # personalization for this participant.
        pconfig = self._load_toml_config(p_config)
        if participant not in pconfig:
            self._logger.warn("{} is not in the participant config file we "
                              "loaded! We can't personalize!".format(
                                   participant))
        elif session not in pconfig[participant]:
            self._logger.warn("Session {} is not in the participant config "
                              "file we loaded for participant {}! We can't "
                              "personalize!".format(session, participant))
        else:
            # Since we are only running one participant and one session, we
            # only care about the configuration for this participant and this
            # session. So we'll only save that information.
            # TODO if we include any higher-level participant config later, we
            # will need that too - then just save this participant and refer to
            # the session as needed.
            self._p_config = pconfig[participant][session]

        # Set up performance logger for tracking participant performance this
        # session.
        self._performance_log = PerformanceLogger(participant, session,
                                                  output_dir)
        # Set up script parser.
        self._script_parser = ScriptParser()
        # These are other script parsers we may use later.
        self._story_parser = None
        self._repeat_parser = None
        # If we have a repeating script, we will need to save its filename so
        # we can re-load it when we repeat it.
        self._repeating_script_name = ""

        # Get session script from script parser and give to the script parser.
        # Story scripts we will get later from the personalization manager.
        try:
            self._script_parser.load_script(
                self._study_path + self._session_script_path
                + self._script_parser.get_session_script(session))
        except IOError as ioe:
            self._logger.exception("Script parser could not open session "
                                   "script! Error: {}".format(ioe))
            # Pass exception up so whoever wanted a script handler knows they
            # didn't get a script.
            raise

        # Initialize flags and counters:
        # Set up counter for how many stories have been told this session.
        self._stories_told = 0

        # We need to track which scene has been selected so we know which story
        # to tell, since it depends on the scene.
        self._selected_scene = None

        # When we start, we are not currently telling a story or repeating a
        # script, or at the end of the game.
        self._doing_story = False
        self._repeating = False
        self._end_game = False

        # For counting repetitions of a repeating script.
        self._repetitions = 0

        # The script will tell us the max number of repetitions.
        self._max_repetitions = 1

        # The script will tell us the max number of stories.
        self._max_stories = 1

        # The maximum number of incorrect user responses before we should move
        # on (can also be set in the script).
        self._max_incorrect_responses = 2

        # Set the maximum game time, in minutes. This can also be set in the
        # interaction script.
        self._max_game_time = datetime.timedelta(minutes=10)

        # Set the time to wait for prompts, in seconds. We may want this to
        # vary for different questions, so it can be set and changed in the
        # script.
        self._prompt_time = 5

        # Number of times we should prompt before moving on. We may want to
        # vary this for different questions, so it can be set and changed in
        # the script.
        self._num_prompts = 2

        # Sometimes we may need to know what the last user response we waited
        # for was, and how long we waited.
        self._last_response_to_get = None
        self._last_response_timeout = None

        # Save start time so we can check whether we've run out of time.
        self._start_time = datetime.datetime.now()

        # Initialize total time paused.
        self._total_time_paused = datetime.timedelta(seconds=0)

        # Initialize pause start time in case someone calls the resume game
        # timer function before the pause game function.
        self._pause_start_time = None

    def _load_toml_config(self, config):
        """ Load in a toml config file for later reference. """
        try:
            with open(config) as tof:
                toml_data = toml.load(tof)
            self._logger.debug("Reading toml config...: {}".format(
                toml_data))
            return toml_data
        except Exception as exc:  # pylint: disable=broad-except
            self._logger.exception("""Could not read your toml config file
                                   \"{}\". Does the file exist? Is it valid
                                   toml? Exiting because we need the config
                                   file to continue. {}""".format(config, exc))
            exit(1)

    def iterate_once(self):
        """ Get the next command from the script and execute it. """
        # Pylint's branch limit is too low in this case; we have a lot of
        # script parsing cases to handle.
        # pylint: disable=too-many-branches
        try:
            # We check whether we've reached the game time limit when we load
            # new stories or when we are about to start a repeating script over
            # again.

            # Get the next line to parse. Scripts are nested: at the top level
            # we're in the main session script. From there, we can enter either
            # a story script or a repeating script. We can also enter a story
            # script from a repeating script, or vice versa.

            # Get next line from story script.
            if self._doing_story and self._story_parser:
                self._logger.debug("Getting next line from story script.")
                line = self._story_parser.next_line()
            # If not in a story, get next line from repeating script.
            elif self._repeating and self._repeat_parser:
                self._logger.debug("Getting next line from repeating script.")
                line = self._repeat_parser.next_line()
            # If not repeating, get next line from main session script.
            else:
                self._logger.debug("Getting next line from main session "
                                   "script.")
                line = self._script_parser.next_line()

            # Make sure we got a line before we try parsing it. We might not
            # get a line if the file has closed or if next_line has some other
            # problem.
            if not line:
                self._logger.warning("[iterate_once] Tried to get next line "
                                     "to parse, but got None!")
                return
            # Okay, we actually do have a line. Parse it! Take action!
            self._parse_line(line)

        # If we get a stop iteration exception instead of a line, we're at the
        # end of the file and will stop iterating over lines.
        except StopIteration:
            # If we were doing a story, now we're done. Go back to the previous
            # script.
            if self._doing_story:
                self._logger.info("Finished story {} of {}!".format(
                    (self._stories_told + 1), self._max_stories))
                self._doing_story = False
                self._stories_told += 1
            # If we were repeating a script, increment counter.
            elif self._repeating:
                self._repetitions += 1
                self._logger.info("Finished repetition {} of {}!".format(
                    self._repetitions, self._max_repetitions))
                # If we've done enough repetitions, or if we've run out of game
                # time, go back to the main session script (set the repeating
                # flag to false).
                if self._repetitions >= self._max_repetitions \
                    or self._end_game \
                    or ((datetime.datetime.now() - self._start_time)
                        - self._total_time_paused >= self._max_game_time):
                    self._logger.info("Done repeating!")
                    self._repeating = False
                # Otherwise, we need to repeat again. Reload the repeating
                # script so we can repeat it.
                else:
                    # Create a script parser for the filename provided.
                    # Assume it is in the session_scripts directory.
                    self._repeat_parser = ScriptParser()
                    try:
                        self._repeat_parser.load_script(
                            self._study_path + self._session_script_path
                            + self._repeating_script_name)
                    except IOError:
                        self._logger.exception("Script parser could not open "
                                               "session script to repeat! "
                                               "Skipping REPEAT line.")
                        self._repeating = False
                        return
            # Otherwise we're at the end of the main script.
            else:
                self._logger.info("No more script lines to get!")
                # Pass on the stop iteration exception.
                raise

        except ValueError:
            # We may get this exception if we try to get the next line but the
            # script file is closed. If that happens, something probably went
            # wrong with ending playback of a story script or a repeating
            # script. End repeating and end the current story so we go back to
            # the main session script.
            if self._doing_story:
                self._doing_story = False
            if self._repeating:
                self._repeating = False

        # Oh no got some unexpected error! Raise it again so we can figure out
        # what happened and deal with it during debugging.
        except Exception as exc:
            self._logger.exception("Unexpected exception! {}".format(exc))
            raise

    def _parse_line(self, line):
        """ Parse a script line and take action. """
        # Pylint seems to think we have too many branches and statements here.
        # One option would be to split the parsing into smaller functions for
        # each type of line encountered (e.g., call a "repeat_script" function
        # when we find a REPEAT line instead of parsing and dealing with it
        # right there), but there's not that much benefit to splitting it right
        # now.
        # pylint: disable=too-many-branches
        # pylint: disable=too-many-statements
        # Got a line - print for debugging.
        self._logger.debug("LINE: " + repr(line))

        # Parse line! Split on tabs.
        elements = line.rstrip().split('\t')
        self._logger.debug("... {} elements: {}".format(
            len(elements), elements))

        # Skip blank lines.
        if len(elements) < 1:
            self._logger.info("Line was empty! Going to next line...")
            return

        # Do different stuff depending on what the first element is.
        #########################################################
        # For STORY lines, play the next story for the participant.
        if len(elements) == 1 and "STORY" in elements[0]:
            self._logger.debug("STORY")
            # If line indicates we need to start a story, do so.
            self._doing_story = True
            # Create a script parser for the filename provided,
            # assuming it is in the story scripts directory.
            self._story_parser = ScriptParser()
            try:
                # Try loading the story for the selected scene.
                self._story_parser.load_script(
                    self._study_path + self._story_script_path +
                    self._p_config["stories"][self._selected_scene])
            except IOError:
                self._logger.exception("Script parser could not open story "
                                       "script! Skipping STORY line.")
                self._doing_story = False
            except AttributeError:
                self._logger.exception("Script parser could not open story "
                                       "script because no script was loaded!"
                                       "Skipping STORY line.")
                self._doing_story = False
            except NoStoryFound:
                self._logger.exception("Script parser could not get the next "
                                       "story script because no script was "
                                       "found by the personalization manager!"
                                       " Skipping STORY line.")
                self._doing_story = False

        # Line has 2+ elements, so check the other commands.
        #########################################################
        # For STORY SETUP lines, pick the next story to play so we can load
        # its graphics and play back the story.
        elif "STORY" in elements[0] and "SETUP" in elements[1]:
            self._logger.debug("STORY SETUP")
            # Pick the next story to play.
            # TODO update story picking and setup
            print "TODO"

        #########################################################
        # For ROBOT lines, send command to the robot.
        elif "ROBOT" in elements[0]:
            self._logger.debug("ROBOT")
            # Send a DO speech or animation playback command to the robot.
            if len(elements) > 2:
                if "DO" in elements[1]:
                    self._send_robot_do(elements[2])
                elif "FIDGET" in elements[1]:
                    if "EMPTY" in elements[2]:
                        self._ros_node.send_tega_command(
                                fidgets=TegaAction.FIDGETS_EMPTY)
                    elif "SPEECH" in elements[2]:
                        self._ros_node.send_tega_command(
                                fidgets=TegaAction.FIDGETS_SPEECH)
                    elif "PHYSICAL" in elements[2]:
                        self._ros_node.send_tega_command(
                                fidgets=TegaAction.FIDGETS_PHYSICAL)

            # Send a different command to the robot.
            elif len(elements) == 2:
                try:
                    # Select a random element from the specified array of robot
                    # action, so that the robot does not always do the same
                    # thing each time.
                    robot_action = self._script_config[elements[1].lower()]
                    self._send_robot_do(
                        robot_action[random.randint(0, len(robot_action - 1))])
                except KeyError:
                    self._logger.warning("{} not in script config!".format(
                        elements[1].lower()))

        #########################################################
        # For OPAL lines, send command to Opal game
        # TODO update what Opal commands we need.
        elif "OPAL" in elements[0]:
            self._logger.debug("OPAL")
            if "LOAD_ALL" in elements[1] and len(elements) >= 3:
                # Load all objects listed in file -- the file is assumed to
                # have properties for one object on each line.
                to_load = self._read_list_from_file(
                    self._study_path + self._session_script_path +
                    elements[2])
                for obj in to_load:
                    self._ros_node.send_opal_command("LOAD_OBJECT", obj)

            # Get the next story and load graphics into the game.
            elif "LOAD_STORY" in elements[1]:
                self._load_next_story()

            # Load answers for game.
            elif "LOAD_ANSWERS" in elements[1] and len(elements) >= 3:
                self._load_answers(elements[2])

            # Send an opal command, with properties.
            elif len(elements) > 2:
                self._ros_node.send_opal_command(elements[1], elements[2])

            # Send an opal command, without properties.
            else:
                self._ros_node.send_opal_command(elements[1])

        #########################################################
        # For PAUSE lines, sleep for the specified number of seconds before
        # continuing script playback.
        elif "PAUSE" in elements[0] and len(elements) >= 2:
            self._logger.debug("PAUSE")
            try:
                time.sleep(int(elements[1]))
            except ValueError:
                self._logger.exception("Not pausing! PAUSE was given an "
                                       "invalid argument. Should be an int!")

        #########################################################
        # For SET lines, set the specified constant.
        elif "SET" in elements[0] and len(elements) >= 3:
            self._logger.debug("SET")
            if "MAX_INCORRECT_RESPONSES" in elements[1]:
                self._max_incorrect_responses = int(elements[2])
                self._logger.info("Set MAX_INCORRECT_RESPONSES to {}".format(
                    elements[2]))
            elif "PROMPT_TIME" in elements[1]:
                self._prompt_time = int(elements[2])
                self._logger.info("Set PROMPT_TIME to {}".format(elements[2]))
            elif "NUM_PROMPTS" in elements[1]:
                self._num_prompts = int(elements[2])
                self._logger.info("Set NUM_PROMPTS to {}".format(elements[2]))
            elif "MAX_GAME_TIME" in elements[1]:
                self._max_game_time = datetime.timedelta(
                    minutes=int(elements[2]))
                self._logger.info("Set MAX_GAME_TIME to {}".format(
                    elements[2]))
            elif "MAX_REPEATS" in elements[1]:
                self._max_stories = int(elements[2])
                self._logger.info("Set MAX_REPEATS to {}".format(elements[2]))

        #########################################################
        # For WAIT lines, wait for the specified user response,
        # or for a timeout.
        elif "WAIT" in elements[0] and len(elements) >= 3:
            self._logger.debug("WAIT")
            self.wait_for_user_tablet_response(elements[1], int(elements[2]))

        #########################################################
        # For QUESTION lines, load and play the specified question, using
        # the script config file question definition.
        elif "QUESTION" in elements[0] and len(elements) >= 2:
            self._logger.info("Current question: " + elements[1])
            self._do_question(elements[1])

        #########################################################
        # For REPEAT lines, repeat lines in the specified script file the
        # specified number of times.
        elif "REPEAT" in elements[0] and len(elements) >= 3:
            self._logger.debug("REPEAT")
            self._repeating = True
            self._repetitions = 0
            # Create a script parser for the filename provided, assume it
            # is in the session_scripts directory.
            self._repeat_parser = ScriptParser()
            self._repeating_script_name = elements[2]
            try:
                self._repeat_parser.load_script(
                    self._study_path + self._session_script_path
                    + elements[2])
            except IOError:
                self._logger.exception("Script parser could not open session "
                                       "script to repeat! Skipping REPEAT.")
                self._repeating = False
                return

            # Figure out how many times we should repeat the script.
            if "MAX_REPEATS" in elements[1]:
                try:
                    self._max_repetitions = self._max_stories
                except AttributeError:
                    self._logger.exception("Tried to set MAX_REPETITIONS to "
                                           "MAX_REPEATS, but MAX_REPEATS has "
                                           "not been set. Defaulting to 1 "
                                           "repetition.")
                    self._max_repetitions = 1
            else:
                self._max_repetitions = int(elements[1])
            self._logger.debug("Going to repeat {} {} time(s).".format(
                elements[2], self._max_repetitions))

    def _read_list_from_file(self, filename):
        """ Read a list of robot responses from a file, return a list of the
        lines from the file.
        """
        # Open script for reading.
        try:
            fih = open(filename, "r")
            return fih.readlines()
        except IOError as ioe:
            self._logger.exception("Cannot open file: {}. Error: {}".format(
                filename, ioe))
            # Pass exception up so anyone trying to add a response list from a
            # script knows it didn't work.
            raise

    def _do_question(self, question):
        """ Given a question for the robot to ask, tell the robot to say it,
        and wait for any user responses required.
        """
        # pylint: disable=too-many-branches
        # Get the question from the script config. It'll have the audio for the
        # question, as well as user input to wait for and check for.
        if "questions" not in self._script_config:
            self._logger.warning("No questions present in script config!")

        question_audio = ""
        user_input = []
        if question in self._script_config["questions"]:
            # Get the audio for the question from the config file.
            question_audio = self._script_config["questions"][question][
                "question"]
            # Get the user responses we should react to from the config file.
            if "user_input" in self._script_config["questions"][question]:
                user_input = self._script_config["questions"][question][
                    "user_input"]
            else:
                self._logger.warning("No user input set for this question!")

        # Tell the robot to play the question.
        self._send_robot_do(question_audio)

        # Wait for a user response or a timeout. If we get a timeout, use a
        # prompt and wait again. Repeat until we have used all the allowed
        # number of prompts have been used, at which point, if we still haven't
        # gotten a valid user response, break and move on.
        # We don't use i in the loop, but you can't really loop without it.
        # pylint: disable=unused-variable
        for i in range(0, self._num_prompts + 1):
            # Announce that it's the user's turn to talk.
            self._ros_node.send_interaction_state(True)
            # Tell ASR node to listen for a response and send us results.
            self._ros_node.send_asr_command(AsrCommand.START_FINAL)
            results = self._ros_node.wait_for_response(
                    self._ros_node.ASR_RESULT,
                    timeout=datetime.timedelta(
                        seconds=int(self._prompt_time)))[0]

            # We either got a response or timed out, so send ASR command to
            # stop listening.
            self._logger.debug("Done waiting on ASR. Timed out or response"
                               ": {}".format(results))
            self._ros_node.send_asr_command(AsrCommand.STOP_ALL)

            # After waiting for a response, we need to play back an appropriate
            # robot response. The robot's response depends on what kind of
            # result we got. There are three options:
            # (1) Nothing, i.e., the ASR gave us nothing back. This is usually
            #     because the user said nothing for the ASR to recognize. If we
            #     are waiting for final ASR results, this is the same as a
            #     TIMEOUT response because we did not get an ASR result in the
            #     specified amount of time. In this case, we play a prompt.
            # (2) Something in the list of user responses that we want to hear.
            #     These user responses are matched up with robot responses in
            #     the script config file, so we play the appropriate robot
            #     response.
            # (3) Something not in the list of expected user responses, which
            #     could be just about anything else, or possibly a wrong
            #     transcription. Play a more generic prompt to see if we can
            #     get an expected response.

            # Case (1): Nothing / Timeout.
            if "TIMEOUT" in results:
                # If we have exhausted our allowed number of prompts, leave the
                # loop if we didn't get a user response.
                if i >= self._num_prompts:
                    break
                self._logger.debug("TIMEOUT: Playing a prompt.")
                # Pick a random prompt from the set of timeout prompts for this
                # question, or, if there are none for this question, from the
                # general list of timeout prompts.
                prompt = ""
                if "timeout_prompts" in self._script_config["questions"][
                        question]:
                    prompt = self._script_config["questions"][question][
                            "timeout_prompts"][random.randint(0, len(
                                self._script_config["questions"][question][
                                    "timeout_prompts"]) - 1)]
                elif "timeout_prompts" in self._script_config:
                    prompt = self._script_config["timeout_prompts"][
                            random.randint(0, len(
                                self._script_config["timeout_prompts"]) - 1)]
                else:
                    self._logger.warning("No timeout prompts found in script"
                                         " config so we cannot play one!")
                    continue
                # Play the selected prompt.
                self._send_robot_do(prompt)
                # Wait for the next user response.
                continue

            # If we got something, we need to parse it and check for Case (2)
            # and Case (3).
            # results = (transcription, confidence)
            # TODO do something with confidence (if low, ask "say again?")
            # We have a list of dictionaries of lists. Each dictionary is one
            # of the expected user response options, which contains a list of
            # things the user might say (e.g., variants on the word "yes" such
            # as "yeah" and "yup") and the robot's response to that. We have to
            # check each one to see if the ASR response matches anything.  Note
            # that here, we may get false positives because we match words
            # against the whole results string, and one of our words might
            # match against part of one of the results word. However, we can't
            # get away with splitting the ASR results into words on spaces
            # because then we can't as easily match phrases. There is probably
            # some cool algorithm involving ngrams and phrase matching that
            # could make this work better, but we're going to use the naive
            # approach for now.
            self._logger.debug("Parsing ASR!")
            for response_option in user_input:
                self._logger.debug("\tcomparing {}".format(response_option))
                for word in response_option["user_responses"]:
                    self._logger.debug("\t\tcomparing {}".format(word))
                    if word in results[0]:
                        # Case (2): Found an expected user response.
                        self._logger.info("Got expected response!")
                        # Play the robot's responses in sequence.
                        for resp in response_option["robot_responses"]:
                            self._send_robot_do(resp)
                        # Return so we don't give the user a chance to
                        # respond again, since we already got their
                        # response and dealt with it.
                        return

            # Case (3): Other user response. The ASR result did not contain
            # any of the expected user responses. Pick a "backchannel" prompt
            # to encourage the user to keep responding in hopes that we will
            # get an expected response, either from the set of prompts specific
            # to this question, or, if there are none, from the general list.
            self._logger.info("Got a response, but not what we wanted...")
            # If we have exhausted our allowed number of prompts, leave the
            # loop if we didn't get a user response instead of playing a
            # backchannel prompt.
            if i >= self._num_prompts:
                break
            if "backchannel_prompts" in self._script_config["questions"][
                    question]:
                prompt = self._script_config["questions"][question][
                        "backchannel_prompts"][random.randint(0, len(
                            self._script_config["questions"][question][
                                "backchannel_prompts"]) - 1)]
            elif "backchannel_prompts" in self._script_config:
                prompt = self._script_config["backchannel_prompts"][
                        random.randint(0, len(
                            self._script_config["backchannel_prompts"]) - 1)]
            else:
                self._logger.warning("No backchannel prompts found in script "
                                     "config so we cannot play one!")
                continue
            # Play the selected prompt.
            self._send_robot_do(prompt)

        # We exhausted our allowed number of prompts, so have the robot do
        # something and move on instead of waiting more.
        # Play the "max attempts reached" robot response, since we used up
        # all the prompts and didn't get an expected user response. Either
        # play a generic one, or, if this question has a specific set of
        # max attempts robot responses, use one of those.
        self._logger.info("Playing max attempts robot response, since we did "
                          "not get a response to the prompts.")
        audio_to_play = ""
        if "max_attempt" in self._script_config["questions"][
                question]:
            audio_to_play = self._script_config["questions"][question][
                    "max_attempt"][random.randint(0, len(
                        self._script_config["questions"][question][
                            "max_attempt"]) - 1)]
        elif "max_attempt" in self._script_config:
            audio_to_play = self._script_config["max_attempt"][
                    random.randint(0, len(
                        self._script_config["max_attempt"]) - 1)]
        else:
            self._logger.warning("No max attempt audio found in script config"
                                 " so we cannot play one!")
        self._send_robot_do(audio_to_play)

    def _send_robot_do(self, command):
        """ Given a command for the robot to do, get any details from the
        script config file and tell the robot to do stuff.
        """
        # If the command is all uppercase, it's just a single animation for the
        # robot to play, so just play it and return.
        if command.isupper():
            self._logger.info("DO animation: {}".format(command))
            self._ros_node.send_tega_command(motion=command, enqueue=True)
            self._ros_node.wait_for_response(self._ros_node.ROBOT_NOT_MOVING,
                                             timeout=datetime.timedelta(
                                                 seconds=int(self.WAIT_TIME)))
            return

        # Check the script config for the audio command. If the command name is
        # listed in the config, then there are some choreographed animations to
        # play with the audio, and/or the audio filename is different from the
        # command name.
        if command == "":
            self._logger.warning("Told to play audio, but the name provided is"
                                 + " an empty string!")
            return

        audio_to_play = command + ".wav"
        animations = []
        if "audio" not in self._script_config:
            self._logger.debug("No audio present in script config!")

        if command in self._script_config["audio"]:
            self._logger.info("Going to play ".format(command))
            audio_to_play = self._script_config["audio"][command]["name"] \
                + ".wav"
            animations = self._script_config["audio"][command]["animations"]

        # Now we have the audio to play and the animations to play.
        # If there are no animations to play, just send the audio command.
        # Otherwise, send the audio command, and tell the ROS node to send the
        # list of animations. Turn on speech fidgets in case it takes a while
        # for the robot to start speaking!
        self._ros_node.send_tega_command(fidgets=TegaAction.FIDGETS_EMPTY)
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
            self._ros_node.send_entrain_audio_message(
                self._audio_base_dir + audio_to_play,
                self._viseme_base_dir + audio_to_play.replace(".wav", ".txt"))
        else:
            # Send directly to the robot.
            self._ros_node.send_tega_command(audio=audio_to_play, enqueue=True)

        # If there are no animations to play during this speech, just wait for
        # the robot to be done playing the audio.
        if len(animations) == 0:
            self._ros_node.wait_for_response(
                self._ros_node.ROBOT_NOT_SPEAKING,
                timeout=datetime.timedelta(seconds=int(self.WAIT_TIME)))
        else:
            # Otherwise, there are animations to play. We don't want to wait
            # for the robot to be done speaking before we send them, so we
            # don't wait for a response. Instead, we start sending animations.
            self._ros_node.wait_for_response(
                self._ros_node.ROBOT_SPEAKING,
                timeout=datetime.timedelta(seconds=int(self.WAIT_TIME)))
            for i in range(0, len(animations)):
                # Wait for the animation's start time. Times are in seconds
                # since the start of the audio file.
                if i == 0:
                    time.sleep(animations[i]["time"])
                else:
                    time.sleep(animations[i]["time"] - animations[i-1]["time"])

                # Play the next animation!
                self._ros_node.send_tega_command(motion=animations[i]["anim"])

            # If we've played all the animatons, wait for the robot to be done
            # speaking before moving on.
            self._ros_node.wait_for_not_speaking()

        # After the robot is done speaking, switch back to physical fidgets.
        self._ros_node.send_tega_command(fidgets=TegaAction.FIDGETS_PHYSICAL)

    def wait_for_user_tablet_response(self, response_to_get, timeout):
        """ Wait for a user response on the tablet, or wait until the specified
        time has elapsed. If the response is incorrect, allow multiple attempts
        up to the maximum number of incorrect responses.
        """
        # TODO What kinds of responses will we be waiting for now?
        # We don't use i in the loop, but you can't really loop without it.
        # pylint: disable=unused-variable
        print "TODO tablet responses"
        """
        for i in range(0, self._max_incorrect_responses):
            self._logger.info("Waiting for user response...")

            # Save the response we were trying to get in case we need to try
            # again.
            self._last_response_to_get = response_to_get
            self._last_response_timeout = timeout
            # Wait for the specified type of response, or until the specified
            # time has elapsed.
            response, answer = self._ros_node.wait_for_response(
                response_to_get, datetime.timedelta(seconds=int(timeout)))

            # After waiting for a response, need to play back an appropriate
            # robot response.

            # If we didn't receive a response, then it was probably because we
            # didn't send a valid response to wait for.  This is different from
            # a TIMEOUT since we didn't time out -- we just didn't get a
            # response of any kind.
            if not response:
                self._logger.info("Done waiting. Did not get valid response!")
                return False

            elif "TIMEOUT" in response:
                # TODO Determine whether we should wait again, or skip waiting
                # for this response.
                self._logger.warning("TODO: TIMEOUT waiting for user response")

            # If response was INCORRECT, randomly select a robot response to an
            # incorrect user action.
            elif "INCORRECT" in response:
                try:
                    # TODO pick robot speech for incorrect response.
                    audio_to_play = ""
                    self._ros_node.send_tega_command(audio=audio_to_play)
                    self._ros_node.wait_for_response(
                        self._ros_node.ROBOT_NOT_SPEAKING,
                        timeout=datetime.timedelta(
                            seconds=int(self.WAIT_TIME)))
                except AttributeError:
                    self._logger.exception("Could not play an incorrect "
                                           "response. Maybe none were loaded?")
                # Don't break so we allow the user a chance to respond again.

            # If response was NO, randomly select a robot response to the user
            # selecting no.
            elif "NO" in response:
                try:
                    # TODO pick robot speech for no responses.
                    audio_to_play = ""
                    self._ros_node.send_tega_command(audio=audio_to_play)
                    self._ros_node.wait_for_response(
                        self._ros_node.ROBOT_NOT_SPEAKING,
                        timeout=datetime.timedelta(
                            seconds=int(self.WAIT_TIME)))
                except AttributeError:
                    self._logger.exception("Could not play a response to user"
                                           " NO. Maybe none were loaded?")
                # Don't break so we allow the user a chance to respond again.

            # If response was CORRECT, randomly select a robot response to a
            # correct user action, highlight the correct answer, and break out
            # of response loop.
            elif "CORRECT" in response:
                try:
                    # TODO pick robot speech for correct responses.
                    audio_to_play = ""
                    self._ros_node.send_tega_command(audio=audio_to_play)
                    self._ros_node.wait_for_response(
                        self._ros_node.ROBOT_SPEAKING,
                        timeout=datetime.timedelta(
                            seconds=int(self.WAIT_TIME)))
                    self._ros_node.send_opal_command("SHOW_CORRECT")
                    self._ros_node.wait_for_response(
                        self._ros_node.ROBOT_NOT_SPEAKING,
                        timeout=datetime.timedelta(
                            seconds=int(self.WAIT_TIME)))

                    # Pause after speaking before hiding correct again
                    time.sleep(self.ANSWER_FEEDBACK_PAUSE_TIME)
                    self._ros_node.send_opal_command("HIDE_CORRECT")
                except AttributeError:
                    self._logger.exception("Could not play a correct response "
                                           "or could not play robot's answer "
                                           "feedback. Maybe none were loaded?")
                # Break from the for loop so we don't give the user a chance to
                # respond again.
                break

            # If response was START, randomly select a robot response to the
            # user selecting START, and break out of response loop.
            elif self._ros_node.START in response:
                try:
                    # TODO pick robot response to START.
                    audio_to_play = ""
                    self._ros_node.send_tega_command(audio=audio_to_play)
                    self._ros_node.wait_for_response(
                        self._ros_node.ROBOT_SPEAKING,
                        timeout=datetime.timedelta(
                            seconds=int(self.WAIT_TIME)))
                except AttributeError:
                    self._logger.exception("Could not play response to"
                                           "user's START. Maybe none were "
                                           "loaded?")
                # Break from the for loop so we don't give the user a
                # chance to respond again.
                break

        # We exhausted our allowed number of user responses, so have the robot
        # do something instead of waiting more.
        else:
            # If user was never correct, play robot's correct answer feedback
            # and show which answer was correct in the game.
            if "CORRECT" in response_to_get:
                try:
                    self._ros_node.send_opal_command("SHOW_CORRECT")
                    # TODO robot response to correct answers.
                    audio_to_play = ""
                    self._ros_node.send_tega_command(audio=audio_to_play)
                    self._ros_node.wait_for_response(
                        self._ros_node.ROBOT_SPEAKING,
                        timeout=datetime.timedelta(
                            seconds=int(self.WAIT_TIME)))

                    # Pause after speaking before hiding correct again.
                    time.sleep(self.ANSWER_FEEDBACK_PAUSE_TIME)
                    self._ros_node.send_opal_command("HIDE_CORRECT")
                except AttributeError:
                    self._logger.exception("Could not play robot's answer "
                                           "feedback! Maybe none were loaded?")

            # If user never selects START (which is used to ask the user if
            # they are ready to play), stop all stories and repeating scripts,
            # continue with main script so we go to the end.
            elif "START" in response_to_get:
                self._repeating = False
                self._doing_story = False

        # We got a user response and responded to it!
        return True
        """

    def skip_wait_for_user_response(self):
        """ Skip waiting for a response; treat the skipped response as a NO or
        INCORRECT response.
        """
        print "TODO skip waiting?"
        """
        # TODO is this function necessary anymore?
        # If the response to wait for was CORRECT or INCORRECT, randomly select
        # a robot response to an incorrect user action.
        if "CORRECT" in self._last_response_to_get:
            try:
                # TODO robot response to incorrect action?
                audio_to_play = ""
                self._ros_node.send_tega_command(audio=audio_to_play,
                                                 enqueue=True)
                self._ros_node.wait_for_response(
                    self._ros_node.ROBOT_SPEAKING,
                    timeout=datetime.timedelta(
                        seconds=int(self.WAIT_TIME)))
            except AttributeError:
                self._logger.exception("Could not play an incorrect response. "
                                       "Maybe none were loaded?")

        # If response to wait for was YES or NO, randomly select a robot
        # response for a NO user action.
        elif "NO" in self._last_response_to_get:
            try:
                # TODO robot response to NO actions?
                audio_to_play = ""
                self._ros_node.send_tega_command(audio=audio_to_play,
                                                 enqueue=True)
                self._ros_node.wait_for_response(
                    self._ros_node.ROBOT_SPEAKING,
                    timeout=datetime.timedelta(
                        seconds=int(self.WAIT_TIME)))
            except AttributeError:
                self._logger.exception("Could not play a response to user's "
                                       "NO. Maybe none were loaded?")
       """

    def set_end_interaction(self):
        """ End the game gracefully -- stop any stories or repeating scripts,
        go back to main session script and finish.
        """
        # For now, we just need to set a flag indicating we should end the
        # game. When we check whether we should load another story or repeat a
        # repeating script, this flag will be used to skip back to the main
        # session script, to the end of the game.
        self._end_game = True

    def pause_interaction_timer(self):
        """ Track how much time we spend paused so when we check whether we
        have reached the max game time, we don't include time spent paused.
        """
        self._pause_start_time = datetime.datetime.now()

    def resume_interaction_timer(self):
        """ Add how much time we spent paused to our total time spent paused.
        """
        # Since this function could theoretically be called before we get a
        # call to pause_game_timer, we have to check that there is a pause
        # start time, and then later, reset it so we can't add the same pause
        # length multiple times to our total pause time.
        if self._pause_start_time:
            self._total_time_paused += \
                datetime.datetime.now() - self._pause_start_time
        # Reset pause start time.
        self._pause_start_time = None

    def wait_for_last_response_again(self):
        """ Wait for the same response that we just waited for again, with the
        same parameters for the response and the timeout.
        """
        return self.wait_for_user_tablet_response(
            self._last_response_to_get,
            self._last_response_timeout)

    def _load_answers(self, answer_list):
        """ Load the answer graphics for this story """
        # We are given a list of words that indicate what the answer options
        # are. By convention, the first word is probably the correct answer;
        # the others are incorrect answers. However, we won't set this now
        # because this convention may not hold.  We expect the SET_CORRECT
        # OpalCommand to be used to set which answers are correct or incorrect.
        # split the list of answers on commas.
        answers = answer_list.strip().split(',')

        # Shuffle answers to display them in a random order.
        random.shuffle(answers)

        # Load in the graphic for each answer.
        for answer in answers:
            toload = {}
            # Remove whitespace from name before using it.
            toload["name"] = answer.strip()
            toload["tag"] = "PlayObject"
            toload["slot"] = answers.index(answer) + 1
            toload["draggable"] = False
            toload["isAnswerSlot"] = True
            self._ros_node.send_opal_command("LOAD_OBJECT", json.dumps(toload))

    # TODO load stories for this game
    def _load_next_story(self):
        """ Get the next story, set up the game scene with scene and answer
        slots, and load scene graphics.
        """
        # If we've told the max number of stories, or if we've reached max game
        # time, don't load another story even though we were told to. Instead,
        # play error message from robot saying we have to be done now.
        if self._stories_told >= self._max_stories \
                or ((datetime.datetime.now() - self._start_time)
                    - self._total_time_paused >= self._max_game_time) \
                or self._end_game:
            self._logger.info("We were told to load another story, but we've "
                              "already played the maximum number of stories or"
                              " we ran out of time! Skipping and ending now.")
            self._doing_story = False
            try:
                # TODO robot response to max stories?
                max_story_response = ""
                self._send_robot_do(max_story_response)
            except AttributeError:
                self._logger.exception("Could not play a max stories reached "
                                       "response. Maybe none were loaded?")
            # We were either told to play another story because a repeating
            # script loads a story and the max number of repetitions is greater
            # than the max number of stories, so more stories were requested
            # than can be played, or because we ran out of time and were
            # supposed to play more stories than we have time for. Either way,
            # stop the repeating script if there is one.
            self._repeating = False
            return

        # Get the details for the next story.
        try:
            print "TODO"
            #scenes, in_order, num_answers = \
                #self._personalization_man.get_next_story_details()
        except NoStoryFound:
            # If no story was found, we can't load the story!
            self._logger.exception("Cannot load story because no story to load"
                                   "was found!")
            self._doing_story = False
            return

        # Set up the story scene in the game.
        setup = {}
        setup["numScenes"] = len(scenes)
        setup["scenesInOrder"] = in_order
        setup["numAnswers"] = num_answers
        self._ros_node.send_opal_command("SETUP_STORY_SCENE",
                                         json.dumps(setup))

        # Load the scene graphics.
        for scene in scenes:
            toload = {}
            toload["name"] = "scenes/" + scene
            toload["tag"] = "PlayObject"
            toload["slot"] = scenes.index(scene) + 1
            if not in_order:
                toload["correctSlot"] = scenes.index(scene) + 1
            toload["draggable"] = False if in_order else True
            toload["isAnswerSlot"] = False
            self._ros_node.send_opal_command("LOAD_OBJECT", json.dumps(toload))

        # Tell the personalization manager that we loaded the story so it can
        # keep track of which stories have been played.
        print "TODO"
        #self._personalization_man.record_story_loaded()
