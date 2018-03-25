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
import random  # For picking robot responses and shuffling answer options
import logging  # Log messages
import rr_commons as rr_commons  # Common constants.
from rr_script_parser import ScriptParser  # Parses scripts
from asr_google_cloud.msg import AsrCommand  # Tell ASR to start/stop.
from r1d1_msgs.msg import TegaAction  # Tell Tega to do different fidgets.
from rr_msgs.msg import UserInput  # Get different user input responses.


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

    def __init__(self, ros_node, session, experimenter,
                 study_path, script_config, story_script_path,
                 session_script_path, pconfig, performance_logger, restart):
        """ Save references to ROS connection and logger, get scripts and
        set up to read script lines.
        """
        # pylint: disable=too-many-arguments
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up script handler...")

        # Get the current experimenter's name.
        self._experimenter_name = experimenter

        # Save reference to our ros node so we can publish messages.
        self._ros_node = ros_node

        # Values for various script and config file paths.
        self._study_path = study_path
        self._story_script_path = story_script_path
        self._session_script_path = session_script_path

        # Save the participant configuration file so we can look up the
        # personalization for this participant.
        self._pconfig = pconfig

        # Save the study config.
        self._script_config = script_config

        # Save the performance logger for tracking participant performance this
        # session.
        self._performance_log = performance_logger

        # Are we restarting at a certain point in the script?
        self._restart_point = restart
        self._looking_for_restart = True if restart else False

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

        # Save the session number as a string for later use in checking the
        # participant config file.
        self._session = str(session)
        # Save the session number as an int, too, since we may need to check
        # the config file for other session information, and that's easier to
        # increment or decrement using an int.
        self._sessionint = session

        # Initialize flags and counters:
        # Set up counter for how many stories have been told this session.
        self._stories_told = 0

        # We need to track which scene has been selected so we know which story
        # to tell, since it depends on the scene.
        self._selected_scene = None

        # When we start, we are not currently telling a story or repeating a
        # script, and the robot is not sleeping. We need to track when the
        # robot is sleeping or doing other things that shouldn't be interrupted
        # so we can make sure it doesn't do anything else during these times.
        self._doing_story = False
        self._repeating = False
        self._robot_busy = False

        # Track whether we just played an exuberance line. Each time the script
        # parser finds a line tagged for exuberance (i.e., ME or LE), it checks
        # the current exuberance. Exuberance scores are updated after each
        # question that is asked. Thus, if there are two exuberance lines in a
        # row (e.g., ME then LE, which is very common since you generally want
        # the user to hear either a ME or LE line), and the line happens to be
        # a question, the exuberance score will be updated and could
        # potentially change from ME to LE or vice versa. If the score changes,
        # then it is possible that both lines tagged ME and LE will be played
        # in a row. This would be weird, since they are usually the same audio
        # paired with different animations. So we need to track whether we have
        # just played an exuberance line or not so that we can skip the second
        # one regardless of whether it matches the exuberance condition.  We
        # track it with two variables: one tracks whether we've hit an
        # exuberance line, the other track whether we played it. Two variables
        # are necessary so we can have multiple pairs of tagged lines in a row.
        # Hit one: right exuberance? Yes. Set both flags. Hit the next: the
        # numbers are equal so skip, reset. Hit the next: We've reset, so check
        # exuberance. Right? No; flag one only. Next line: right? Yes.
        # Play it and reset both values. Hit a line with no exuberance? Reset
        # both values.
        #
        # This is a hack. The right solution may be to change the script format
        # or something, but because I would like to graduate we are going with
        # the hack that is faster to write.
        self._exuberance_line_hit = False
        self._exuberance_line_played = False

        # We are also not actively trying to pick a scene to play for a story,
        # and the robot didn't pick it.
        self._picking_scene = False
        self._robot_picked_scene = False

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

        # Set the maximum time for each iteration of a repeating script, in
        # minutes. This can also be set in the interaction script.
        self._max_repeat_time = datetime.timedelta(minutes=10)

        # Set the time to wait for prompts, in seconds. We may want this to
        # vary for different questions, so it can be set and changed in the
        # script.
        self._prompt_time = 5

        # Number of times we should prompt before moving on. We may want to
        # vary this for different questions, so it can be set and changed in
        # the script.
        self._num_prompts = 2

        # Sometimes we may need to know what the last user response we waited
        # for was, how long we waited, and whether we got a response (or timed
        # out). In a couple cases, we need to track what response we got from a
        # user input form so we can take appropriate action later.
        self._last_response_to_get = None
        self._last_response_timeout = None
        self._got_a_response = False
        self._user_input_response = "none"

        # Save start time so we can check whether we've run out of time.
        self._start_time = datetime.datetime.now()
        # We don't start in a repeating script so there's no start time yet.
        self._repeat_start_time = None

        # Initialize time paused counters.
        self._total_time_paused = datetime.timedelta(seconds=0)
        self._repeat_time_paused = datetime.timedelta(seconds=0)

        # Initialize pause start time in case someone calls the resume game
        # timer function before the pause game function.
        self._pause_start_time = None

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
                # If we've done enough repetitions, or if we've run out of
                # repeat time, or if we've run out of game time, go back to the
                # main session script (set the repeating flag to false).
                if self._repetitions >= self._max_repetitions \
                    or ((datetime.datetime.now() - self._repeat_start_time)
                        - self._repeat_time_paused >= self._max_repeat_time) \
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

    def _parse_line_tags(self, tags):
        """ Based on the line tags and the information provided in the
        participant config file, determine whether or not to play this line.
        """
        if "condition" not in self._pconfig:
            self._logger.warning("No condition listed for participant! Can't "
                                 "check tags without this. Skipping line.")
            return False

        if self._pconfig["condition"] not in tags:
            self._logger.info("Wrong condition. Skipping line.")
            return False

        self._logger.info("Right condition...")

        # Check if this is an exuberance line. In general, we only do
        # exuberance if this is a personalized/relational robot, but this is
        # specified in the tag.
        if "ME" in tags or "LE" in tags:
            # If we did not just play an exuberance line, then we
            # can play this one, assuming the user has the
            # appropriate exuberance level.
            if ((self._exuberance_line_hit and
                    not self._exuberance_line_played) or
                    (not self._exuberance_line_hit and
                        not self._exuberance_line_played)) and \
                    self._performance_log.get_exuberance() in tags:
                self._logger.debug("Right exuberance! Doing line.")
                # Remove the tag and parse the line as usual.
                self._exuberance_line_played = True
                self._exuberance_line_hit = not self._exuberance_line_hit
                return True
            # Otherwise, we already played an exuberance line, or
            # we didn't, but this is the wrong exuberance level.
            else:
                self._logger.debug("Wrong exuberance, or we just played an "
                                   "exuberance line. Skipping line.")
                # If we had hit a line before that we played, reset
                # so we can hit the next pair even if it's the next
                # thing.
                self._exuberance_line_played = False
                self._exuberance_line_hit = not self._exuberance_line_hit
                return False

        # Not an exuberance tag; set flags accordingly.
        self._exuberance_line_hit = False
        self._exuberance_line_played = False

        # Check for other tags.
        prev_session = str(self._sessionint - 1)
        # Line can be tagged "SL" or "SD" to indicate that the participant
        # either liked or didn't like the robot's story last time.
        if ("SL" in tags or "SD" in tags) and prev_session in self._pconfig:
            if "liked_story" not in self._pconfig[str(self._sessionint - 1)]:
                self._logger.warning("No liked_story listed for participant "
                                     "for session {}! Can't check tags without"
                                     "this. Skipping line.".format(
                                         self._sessionint - 1))
                return False

            if self._pconfig[prev_session]["liked_story"] not in tags:
                self._logger.debug("Wrong liked_story tag. Skipping line.")
                return False
            else:
                self._logger.debug("Right liked_story tag. Doing line.")
                return True

        if ("TY" in tags or "TN" in tags) and prev_session in self._pconfig:
            if "told_story" not in self._pconfig[prev_session]:
                self._logger.warning("No told_story listed for participant "
                                     "for session {}! Can't check tags without"
                                     "this. Skipping line.".format(
                                         self._sessionint - 1))
                return False

            if self._pconfig[prev_session]["told_story"] not in tags:
                self._logger.debug("Wrong told_story tag. Skipping line.")
                return False
            else:
                self._logger.debug("Right told_story tag. Doing line.")
                return True

        # If we get here, it means we already checked all the other tags that
        # may also be on this line, but there aren't any, so it's just a
        # condition tag, so we do the line since it's the right condition.
        return True

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

        # Skip comments.
        if line.startswith("#"):
            self._logger.info("Line was comment. Going to next line...")
            return

        # Parse line! Split on tabs.
        elements = line.rstrip().split('\t')
        self._logger.debug("... {} elements: {}".format(
            len(elements), elements))

        # If we are looking for a RESTART point, then skip all lines except for
        # RESTART lines and SET lines.
        if self._looking_for_restart and len(elements) > 0:
            # Check all RESTART lines to see if this is our restart point.
            if "RESTART" in elements[0] and len(elements) > 1:
                # If this is our restart point, stop looking at return to
                # parsing and executing lines normally.
                if self._restart_point in elements[1]:
                    self._logger.info("Found restart point!")
                    self._looking_for_restart = False
                return
            # We should execute all SET lines, untagged ROBOT VOLUME lines, and
            # any STATE lines that mention the <condition>. Skip all others.
            elif "SET" in elements[0] or \
                    ("VOLUME" in line and not line.startswith("**")) or \
                    "<condition>" in line:
                self._logger.debug("... looking for restart, but should do "
                                   "this line!")
            else:
                self._logger.debug("... looking for restart; skipping!")
                return

        # If this line is tagged with a phrase (e.g., "RR" or "NR" for
        # condition, or "ME" or "LE" for more/less exuberant) do the line only
        # if the participant has the same tag (i.e., is in the "RR" or "NR"
        # condition, and/or is more/less exuberant).
        if line.startswith("**") and len(elements) > 1:
            self._logger.info("Line is tagged. Checking to see if we do it...")
            if self._parse_line_tags(elements[0]):
                # Remove the tag and parse the line as usual.
                self._logger.info("Right tags - doing line!")
                del elements[0]
            else:
                self._logger.info("Wrong tags - skipping line!")
                return

        else:
            # There was no tag on this, so we can clear the exuberance flag.
            self._exuberance_line_hit = False
            self._exuberance_line_played = False

        # If this is an IF RESPONSE line, only do the line if the last question
        # got a response (i.e. did not time out).
        if len(elements) > 1 and "IF_RESPONSE" in elements[0]:
            self._logger.debug("IF_RESPONSE")
            if self._got_a_response:
                # Remove the tag and parse the line as usual.
                del elements[0]
                self._logger.info("Got a response last time, doing line!")

                # Check whether this IF RESPONSE line depends on the type of
                # response gotten (currently, positive or negative)
                if len(elements) > 1 and "RESPONSE_" in elements[0]:
                    if self._user_input_response in elements[0]:
                        # Remove tag and parse line as usual.
                        del elements[0]
                        self._logger.info("Needed a certain response; got it!")
                    else:
                        self._logger.info("Didn't get the needed response! "
                                          "Skipping line...")
                        return
            else:
                self._logger.info("Did not get a response last time, "
                                  "skipping line...")
                return

        # Skip blank lines.
        if len(elements) < 1:
            self._logger.info("Line was empty! Going to next line...")
            return

        # Do different stuff depending on what the first element is.
        #########################################################
        # For STORY lines, play the next story for the participant.
        if len(elements) == 1 and "STORY" in elements[0]:
            self._logger.debug("STORY")
            # The line indicates we need to start a story!
            self._doing_story = True
            self._load_next_story_script()

        # Line has 2+ elements, so check the other commands.
        #########################################################
        # For ROBOT lines, send command to the robot.
        elif "ROBOT" in elements[0]:
            self._logger.debug("ROBOT")
            if len(elements) > 2:
                # Send a DO speech or animation playback command to the robot.
                if "DO" in elements[1]:
                    self._send_robot_do(elements[2])
                elif "LOOKAT" in elements[1]:
                    try:
                        self._logger.info("Sending lookat {}".format(
                            elements[2]))
                        self._ros_node.send_tega_command(
                            enqueue=True,
                            lookat=rr_commons.LOOKAT[elements[2]])
                    except KeyError as keyerr:
                        self._logger.warning("Told to send lookat {}, but it's"
                                             "not the presets list! {}".format(
                                              elements[2], keyerr))
                elif "VOLUME" in elements[1]:
                    self._logger.info("Setting robot volume to {}".format(
                        elements[2]))
                    self._ros_node.send_tega_command(
                        enqueue=True,
                        volume=float(elements[2]))
                # Send a fidget command to the robot.
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
                        robot_action[random.randint(0, len(robot_action) - 1)])
                except KeyError:
                    self._logger.warning("{} not in script config!".format(
                        elements[1].lower()))

        #########################################################
        # For STATE lines, send an interaction state message with the contents
        # of the line.
        elif "STATE" in elements[0] and len(elements) >= 2:
            self._logger.debug("STATE")
            if "<condition>" in elements[1]:
                if "condition" in self._pconfig:
                    self._ros_node.send_interaction_state(
                        state=self._pconfig["condition"])
                else:
                    self._ros_node.send_interaction_state(
                        state="no condition set")
                    self._logger.warning("Told to send STATE message with the "
                                         "condition, but no condition is set "
                                         "in the participant config file!")
            else:
                self._ros_node.send_interaction_state(state=elements[1])

        #########################################################
        # For OPAL lines, send command to Opal game.
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

            elif "PICK_STORY" in elements[1]:
                self._picking_scene = True
                self._load_scenes_to_pick()

            # Get the next story and load graphics into the game.
            elif "LOAD_STORY" in elements[1]:
                if not self._done_telling_stories():
                    self._load_next_story_graphics()

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
            if "PROMPT_TIME" in elements[1]:
                self._prompt_time = int(elements[2])
                self._logger.info("Set PROMPT_TIME to {}".format(elements[2]))
            elif "NUM_PROMPTS" in elements[1]:
                self._num_prompts = int(elements[2])
                self._logger.info("Set NUM_PROMPTS to {}".format(elements[2]))
            elif "BACKCHANNEL" in elements[1]:
                backchannel = True if "ON" in elements[2] else False
                story = True if "STORY" in elements[2] else False
                self._ros_node.enable_backchanneling(backchannel, story=story)
                self._logger.info("Set BACKCHANNEL to {}".format(backchannel))
            elif "MAX_GAME_TIME" in elements[1]:
                self._max_game_time = datetime.timedelta(
                    minutes=int(elements[2]))
                self._logger.info("Set MAX_GAME_TIME to {}".format(
                    elements[2]))
            elif "MAX_REPEATS" in elements[1]:
                self._max_stories = int(elements[2])
                self._logger.info("Set MAX_REPEATS to {}".format(elements[2]))
            elif "MAX_REPEAT_TIME" in elements[1]:
                self._max_repeat_time = datetime.timedelta(
                    minutes=int(elements[2]))
                self._logger.info("Set MAX_REPEAT_TIME to {}".format(
                    elements[2]))
            elif "MAX_INCORRECT_RESPONSES" in elements[1]:
                self._max_incorrect_responses = int(elements[2])
                self._logger.info("Set MAX_INCORRECT_RESPONSES to {}".format(
                    elements[2]))

        #########################################################
        # For WAIT lines, wait for the specified user response, which may be
        # from a GUI form (i.e. a ros message) or for a timeout (i.e. no user
        # response).
        elif "WAIT" in elements[0] and len(elements) >= 4:
            self._logger.debug("WAIT")
            # The second and third elements specify what kind of user response
            # to wait for. The fourth element specifies how long to wait before
            # moving on.
            if "USER_INPUT" in elements[1]:
                # Wait for user input from a GUI.
                self._wait_for_user_input(elements[2], int(elements[3]))
            else:
                self._logger.warning("Told to WAIT for {} {} which we don't "
                                     "handle yet!".format(elements[2],
                                                          elements[3]))

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
            self._repeat_start_time = datetime.datetime.now()
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

        # For determining what audio to play for different responses.
        self._got_a_response = False
        question_audio = ""
        user_input = []
        tablet_input = []
        response_to_wait_for = None
        using_asr = False
        if question in self._script_config["questions"]:
            # Get the audio for the question from the config file.
            question_audio = self._script_config["questions"][question][
                "question"]
            # Get the user responses we should react to from the config file.
            # This is only set if we need ASR responses.
            if "user_input" in self._script_config["questions"][question]:
                user_input = self._script_config["questions"][question][
                    "user_input"]
                # We will be waiting for ASR results for the user input.
                using_asr = True
                response_to_wait_for = self._ros_node.ASR_RESULT
            else:
                self._logger.warning("No ASR user input set for the question!")

            # Get the tablet responses we should react to from the config file.
            # This is only set if we need tablet responses.
            if "tablet_input" in self._script_config["questions"][question]:
                tablet_input = self._script_config["questions"][question][
                    "tablet_input"]
                # If there is no response to wait for set yet, then we only
                # need to wait for tablet responses. Otherwise, we need to
                # wait for EITHER a tablet response or an ASR response.
                if response_to_wait_for:
                    response_to_wait_for = self._ros_node.ASR_OR_TABLET
                else:
                    response_to_wait_for = self._ros_node.TABLET_RESPONSE
            else:
                self._logger.warning("No tablet user input set for this "
                                     "question!")

        # Tell the robot to play the question.
        self._send_robot_do(question_audio)

        if not response_to_wait_for:
            self._logger.warning("No input of any kind set for this question!"
                                 " We can't wait for a response if there is no"
                                 " response to wait for! Skipping the rest of"
                                 " the question!")
            return

        # For logging number of prompts needed for the question.
        prompts_played = 0
        max_attempt_hit = False
        latencies = []

        # Wait for a user response or a timeout. If we get a timeout, use a
        # prompt and wait again. Repeat until we have used all the allowed
        # number of prompts have been used, at which point, if we still haven't
        # gotten a valid user response, break and move on.
        # We don't use i in the loop, but you can't really loop without it.
        # pylint: disable=unused-variable
        for i in range(0, self._num_prompts + 1):
            # Announce that it's the user's turn to talk or act.
            # If the robot is not sleeping, do actions...
            if not self._robot_busy:
                # Turn backchanneling on while it's their turn.
                self._ros_node.send_interaction_state(is_user_turn=True)
                self._ros_node.enable_backchanneling(True)
                # Since it's the user's turn to talk or act, the robot should
                # look at the user.
                self._ros_node.send_tega_command(
                    enqueue=True,
                    lookat=rr_commons.LOOKAT["USER"])
            if using_asr:
                # Tell ASR node to listen for a response and send us results.
                #self._ros_node.send_asr_command(AsrCommand.START_FINAL)
                # TODO HACK Using offline ASR.
                self._ros_node.send_offline_asr_command("start continuous")
            # Wait for a response and get results! The first part of the tuple
            # is the response, the second part is optionally information about
            # the response type (e.g. ASR or tablet response), and the third is
            # the time elapsed waiting for the response.
            results, response_type, waited = self._ros_node.wait_for_response(
                    response_to_wait_for,
                    timeout=datetime.timedelta(
                        seconds=int(self._prompt_time)))

            # We either got a response or timed out, so send ASR command to
            # stop listening if we were using ASR. It is also no longer the
            # user's turn and we should stop backchanneling.
            if using_asr:
                # TODO HACK Using offline ASR.
                #self._ros_node.send_asr_command(AsrCommand.STOP_ALL)
                self._ros_node.send_offline_asr_command("stop")
            self._ros_node.send_interaction_state(is_user_turn=False)
            self._ros_node.enable_backchanneling(False)
            self._logger.debug("Done waiting. Timed out or got response"
                               ": {}".format(results))

            # Log the response latency.
            latencies.append(waited.total_seconds())

            # After waiting for a response, we need to play back an appropriate
            # robot response. The robot's response depends on what kind of
            # result we got. There are three options:
            # (1) Nothing, i.e., the ASR gave us nothing back. This is usually
            #     because the user said nothing for the ASR to recognize. If we
            #     are waiting for final ASR results, this is the same as a
            #     TIMEOUT response because we did not get an ASR result in the
            #     specified amount of time. In this case, we play a prompt.
            #     For the tablet, this option means the user took no action on
            #     the tablet, so we got no response.
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
                prompts_played += 1
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
            # ASR results = (transcription, confidence)
            # Tablet results = (touched object, position)
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
            # If we got a tablet response:
            if self._ros_node.TABLET_RESPONSE in response_type:
                self._logger.debug("Parsing tablet response!")
                self._got_a_response = True
                for resp_option in tablet_input:
                    self._logger.debug("\tcomparing {}".format(resp_option))
                    for word in resp_option["user_responses"]:
                        self._logger.debug("\t\tcomparing {}".format(word))
                        if word in results[0]:
                            # Case (2): Found an expected user response.
                            self._logger.info("Got expected response!")
                            # If we are trying to pick a scene, save the name
                            # of the touched object from the tablet as the
                            # selected scene.
                            if self._picking_scene:
                                self._selected_scene = results[0].replace("Scene", "")
                                self._picking_scene = False
                                self._robot_picked_scene = False
                            # Play the robot's responses in sequence.
                            for resp in resp_option["robot_responses"]:
                                self._send_robot_do(resp)
                            # Log the data from this question.
                            self._performance_log.log_question(
                                    self._num_prompts, prompts_played,
                                    max_attempt_hit, latencies)
                            # Return so we don't give the user a chance to
                            # respond again, since we already got their
                            # response and dealt with it.
                            return
            else:
                # If we got an ASR response:
                self._logger.debug("Parsing ASR response!")
                self._got_a_response = True
                for resp_option in user_input:
                    self._logger.debug("\tcomparing {}".format(resp_option))
                    for word in resp_option["user_responses"]:
                        self._logger.debug("\t\tcomparing {}".format(word))
                        if word in results[0]:
                            # Case (2): Found an expected user response.
                            self._logger.info("Got expected response!")
                            # Play the robot's responses in sequence.
                            for resp in resp_option["robot_responses"]:
                                self._send_robot_do(resp)
                            # Log the data from this question.
                            self._performance_log.log_question(
                                    self._num_prompts, prompts_played,
                                    max_attempt_hit, latencies)
                            # If we were picking a story and got here, it means
                            # the child said some expected ASR thing that means
                            # the robot should pick the scene.
                            if self._picking_scene:
                                self._robot_pick_story_scene()
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

        # If we were trying to get the user to pick a scene, if we get here, we
        # did not get the user to pick one. So we need to select a scene
        # ourselves... pick one randomly.
        if self._picking_scene:
            self._robot_pick_story_scene()

        # Play the "max attempts reached" robot response, since we used up
        # all the prompts and didn't get an expected user response. Either
        # play a generic one, or, if this question has a specific set of
        # max attempts robot responses, use one of those.
        max_attempt_hit = True
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
        # Log the data from this question and update the exuberance score.
        self._performance_log.log_question(self._num_prompts, prompts_played,
                                           max_attempt_hit, latencies)

    def _robot_pick_story_scene(self):
        """ If we were trying to get the user to pick a scene, if we get here,
        we did not get the user to pick one. So we need to select a scene
        ourselves... pick one randomly.
        """
        if self._picking_scene:
            if self._session in self._pconfig:
                if "scenes" in self._pconfig[self._session]:
                    self._selected_scene = self._pconfig[self._session][
                            "scenes"][random.randint(
                                0, len(self._pconfig[self._session][
                                    "scenes"]) - 1)]
                    self._logger.info("No user scene selection made! Selecting"
                                      " a scene... {}".format(
                                          self._selected_scene))
                    # Play back speech for the robot picking a scene so the
                    # child knows which was picked. This way the negotiation
                    # can happen normally.
                    # "Let's do the... playground"
                    self._send_robot_do("LetsDoThe")
                    self._send_robot_do(self._selected_scene)

                else:
                    self._logger.error("We were supposed to pick a scene to "
                                       "play, but there are none in the"
                                       "participant config file for this "
                                       "session!")
                self._picking_scene = False
                self._robot_picked_scene = True
            else:
                self._logger.error("We were supposed to pick a scene to play, "
                                   "but there is no session {} in the "
                                   "participant config file!".format(
                                    self._session))

    def _send_robot_do(self, command):
        """ Given a command for the robot to do, get any details from the
        script config file and tell the robot to do stuff.
        """
        # If the command is all uppercase, it's just a single animation for the
        # robot to play, so just play it and return.
        if command.isupper():
            self._logger.info("DO animation: {}".format(command))
            if "SLEEPING" in command or "POSE" in command:
                self._robot_busy = True
            else:
                self._robot_busy = False
            self._ros_node.send_tega_command(motion=command, enqueue=True)
            self._ros_node.wait_for_response(self._ros_node.ROBOT_NOT_MOVING,
                                             timeout=datetime.timedelta(
                                                 seconds=int(self.WAIT_TIME)))
            return

        # If the command contains the keyword "experimenter_name", replace the
        # command with the name of the current experimenter (which we may get
        # from the comamnd line during program startup; if we don't, the
        # variable will be an empty string).
        if "<experimenter_name>" in command:
            command = self._experimenter_name
            self._logger.info("Told to play experimenter name: {}".format(
                self._experimenter_name))

        # If the command contains the keyword "participant_name", replace the
        # command with the name of the current participant, which may be in the
        # participant config file.
        if "<participant_name>" in command:
            if "name" in self._pconfig:
                command = self._pconfig["name"]
            self._logger.info("Told to play participant name: {}".format(
                command))

        # Don't play if there was nothing given to play.
        if command == "":
            self._logger.warning("Told to play audio, but the name provided is"
                                 + " an empty string!")
            return

        # Check the script config for the audio command. If the command name is
        # listed in the config, then there are some choreographed animations to
        # play with the audio, and/or the audio filename is different from the
        # command name.
        audio_to_play = command + ".wav"
        animations = []
        if "audio" not in self._script_config:
            self._logger.debug("No audio present in script config!")

        if command in self._script_config["audio"]:
            if "name" not in self._script_config["audio"][command] or \
                    "animations" not in self._script_config["audio"][command]:
                self._logger.warning("Told to play \"{}\", but it is not "
                                     "configured properly in script config!")
                return

            self._logger.info("Going to play ".format(command))
            audio_to_play = self._script_config["audio"][command]["name"] \
                + ".wav"
            animations = self._script_config["audio"][command]["animations"]

        # Now we have the audio to play and the animations to play.
        # If there are no animations to play, just send the audio command.
        # Otherwise, send the audio command, and tell the ROS node to send the
        # list of animations. Turn on speech fidgets in case it takes a while
        # for the robot to start speaking!
        self._robot_busy = False
        self._ros_node.send_tega_command(fidgets=TegaAction.FIDGETS_SPEECH)
        self._ros_node.send_speech(audio_to_play)

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
                # If it starts with "LOOKAT" then it's actually a lookat
                # command and not an animation.
                if not animations[i]["anim"].startswith("LOOKAT_"):
                    self._ros_node.send_tega_command(
                            motion=animations[i]["anim"])
                else:
                    # Remove the "LOOKAT_" and send the lookat command.
                    try:
                        self._ros_node.send_tega_command(
                                lookat=rr_commons.LOOKAT[
                                    animations[i]["anim"][7:]])
                    except KeyError as keyerr:
                        self._logger.warning("LOOKAT command {} not in our "
                                             "list of presets! {}".format(
                                                 animations[i]["anim"][7:],
                                                 keyerr))

            # If we've played all the animatons, wait for the robot to be done
            # speaking before moving on.
            self._ros_node.wait_for_not_speaking()

        # After the robot is done speaking, switch back to physical fidgets.
        self._ros_node.send_tega_command(fidgets=TegaAction.FIDGETS_PHYSICAL)

    def _wait_for_user_input(self, response_type, timeout):
        """ Wait for user input from a GUI form, or wait until the specified
        time as elapsed.
        """
        if "YESNO" in response_type:
            self._wait_for_yesno(timeout)
        elif "NEGOTIATE" in response_type:
            self._wait_for_negotiate(timeout)
        else:
            self._logger.error("Told to wait for user input, but the type to "
                               "wait for was \"{}\" and we don't handle that"
                               " yet!".format(response_type))

    def _wait_for_yesno(self, timeout):
        """ Steps to take if we need to wait for UserInput on a YESNO line.
        """
        # Robot response to send.
        resp = ""
        # Wait for a button press from the form, or until we exhaust our
        # available number of prompts.
        for i in range(0, self._num_prompts + 1):
            # Wait for a button press from the form.
            # The response is a tuple with the response, possibly followed
            # by a string indicating what type of response this was.
            response = self._ros_node.wait_for_response(
                    self._ros_node.USER_INPUT_YESNO,
                    timeout=datetime.timedelta(seconds=timeout))[0]

            # We either got a response or timed out.
            self._logger.debug("Done waiting for user input. Timed out or "
                               "response: {}".format(response))
            # Response could be:
            # 1) no response (timeout)
            #      -> prompt again
            # 2) positive (yes let's do another picture)
            #      -> record response so we will play the next picture
            # 3) negative (no let's not do another picture)
            #      -> record response so we will skip the next picture

            # Case (1): Nothing / Timeout.
            if UserInput.TIMEOUT in response:
                # If we have exhausted our allowed number of prompts, leave the
                # loop if we didn't get a user response.
                # Log the outcome so it can be referenced next time.
                if i >= self._num_prompts:
                    self._performance_log.log_extra_picture_outcome(response)
                    # Play some final speech that wraps this up.
                    resp = self._get_response_from_config(
                            "extra_picture_no_response")
                    if resp:
                        # Play the selected response.
                        self._send_robot_do(resp)

                # Pick one of a set of possible timeout prompts.
                resp = self._get_response_from_config(
                        "extra_picture_timeout")
                if resp:
                    # Play the selected prompt.
                    self._send_robot_do(resp)
                # Wait for the next user response.
                continue

            # Log the outcome of the extra pic so it can be referenced later.
            self._performance_log.log_extra_picture_outcome(response)

            # Case (2): Positive.
            if UserInput.YES in response:
                self._logger.debug("Outcome: YES - do picture!")
                self._user_input_response = "YES"
                resp = self._get_response_from_config("extra_picture_yes")
                # Break so we don't give the user a chance to respond again,
                # since we already got a response and dealt with it.
                break
            # Case (3): Negative.
            elif UserInput.NO in response:
                self._logger.debug("Outcome: NO - don't do picture!")
                self._user_input_response = "NO"
                # Play one of the posssible acquiescence responses.
                resp = self._get_response_from_config("extra_picture_no")
                # Break so we don't give the user a chance to respond again,
                # since we already got a response and dealt with it.
                break
        # If we get here, we got a response, found the thing the robot should
        # say in reply, logged relevant information about the outcome of the
        # activity, and just need the robot to actually do something!
        self._got_a_response = True
        if resp:
            # Play the selected prompt.
            self._send_robot_do(resp)

    def _wait_for_negotiate(self, timeout):
        """ Steps to take if we need to wait for UserInput on a NEGOTIATE line.
        """
        # Robot response to send.
        resp = ""
        # Wait for a button press from the form, or until we exhaust our
        # available number of prompts.
        for i in range(0, self._num_prompts + 1):
            # Wait for a button press from the form.
            # The response is a tuple with the response, possibly followed
            # by a string indicating what type of response this was.
            response = self._ros_node.wait_for_response(
                    self._ros_node.USER_INPUT_NEGOTIATION,
                    timeout=datetime.timedelta(seconds=timeout))[0]

            # We either got a response or timed out.
            self._logger.debug("Done waiting for user input. Timed out or "
                               "response: {}".format(response))
            # Response could be:
            # 1) no response (timeout)
            #      -> prompt again
            # 2) refusal (no I want to do the one I picked)
            #      -> go along with it, record to reference next session
            # 3) acquiescence (okay we'll do your choice)
            #      -> go along with it, record to reference next session
            # 4) compromise (do both / do mine first / do yours first)
            #      -> do appropriate suggested action, record to reference
            #         next session
            #

            # Case (1): Nothing / Timeout.
            if UserInput.TIMEOUT in response:
                # If we have exhausted our allowed number of prompts, leave the
                # loop if we didn't get a user response.
                # Log the outcome so it can be referenced next time.
                if i >= self._num_prompts:
                    self._performance_log.log_negotiation_outcome(response,
                                                                  "none")
                    # Play some final speech that wraps this up.
                    resp = self._get_response_from_config(
                            "negotiation_no_response")
                    if resp:
                        # Play the selected response.
                        self._send_robot_do(resp)

                # Pick one of a set of possible negotiation timeout prompts.
                resp = self._get_response_from_config(
                        "negotiation_timeout_prompts")
                if resp:
                    # Play the selected prompt.
                    self._send_robot_do(resp)
                # Wait for the next user response.
                continue

            # Log the outcome of the negotiation so it can be referenced later.
            self._performance_log.log_negotiation_outcome(response,
                                                          self._selected_scene)

            # Case (2): Refusal.
            if UserInput.REFUSAL in response:
                # Play one of the posssible negotiation refusal responses.
                self._logger.debug("Negotiation outcome: REFUSAL - do child's "
                                   "choice!")
                resp = self._get_response_from_config("negotiation_refusal")
                # If the robot had picked a scene but the child wants the other
                # one, a refusal means we switch to the child's choice.
                if self._robot_picked_scene:
                    self._switch_selected_scene()
                # Break so we don't give the user a chance to respond again,
                # since we already got a response and dealt with it.
                break

            # Case (3): Acquiescence.
            if UserInput.ACQUIESCENCE in response:
                # Play one of the posssible negotiation acquiescence responses.
                self._logger.debug("Negotiation outcome: ACQUIESENCE - do "
                                   "robot's choice!")
                # Change selected scene to be the other one available, but only
                # if the child actually picked something (otherwise the child
                # is allowing the robot's choice, which was already selected).
                if not self._robot_picked_scene:
                    self._switch_selected_scene()
                resp = self._get_response_from_config("negotiation_acquiesce")
                # Break so we don't give the user a chance to respond again,
                # since we already got a response and dealt with it.
                break

            # Case (4): Compromise. There are three compromise outcomes:
            if UserInput.COMPROMISE_GENERAL in response:
                self._logger.info("Negotiation outcome: GENERAL COMPROMISE - "
                                  "do child's choice!")
                resp = self._get_response_from_config("negotiation_general")
                # Break so we don't give the user a chance to respond again,
                # since we already got a response and dealt with it.
                break

            if UserInput.COMPROMISE_ROBOT in response:
                self._logger.info("Negotiation outcome: ROBOT COMPROMISE - "
                                  "do robot's choice!")
                # Change selected scene to be the other one available, if the
                # child had initially picked a scene.
                if not self._robot_picked_scene:
                    self._switch_selected_scene()
                resp = self._get_response_from_config("negotiation_general")
                # Break so we don't give the user a chance to respond again,
                # since we already got a response and dealt with it.
                break

            if UserInput.COMPROMISE_CHILD in response:
                self._logger.info("Negotiation outcome: CHILD COMPROMISE - "
                                  "do child's choice!")
                resp = self._get_response_from_config("negotiation_general")
                # If the robot had picked the scene and the child wants the
                # other one, switch scenes.
                if self._robot_picked_scene:
                    self._switch_selected_scene()
                # Break so we don't give the user a chance to respond again,
                # since we already got a response and dealt with it.
                break

        # If we get here, we got a response, found the thing the robot should
        # say in reply, logged relevant information about the outcome of the
        # negotiation, and just need the robot to actually do something!
        self._got_a_response = True
        if resp:
            # Play the selected prompt.
            self._send_robot_do(resp)

    def _switch_selected_scene(self):
        """ Switch the selected scene for a scene from the participant config's
        list of scenes that wasn't selected.
        """
        if self._session in self._pconfig:
            if "scenes" in self._pconfig[self._session]:
                for scene in self._pconfig[self._session]["scenes"]:
                    if scene != self._selected_scene:
                        self._logger.info("Changing selected scene to "
                                          "{}".format(scene))
                        self._selected_scene = scene
                        break
            else:
                self._logger.error("No scenes in participant config? Not sure "
                                   "how we picked a scene before if none were "
                                   "available to pick from. Not switching "
                                   "scene!")
        else:
            self._logger.error("We were supposed to switch the selected scene,"
                               " but there is no session {} in the participant"
                               "config file with a scene list!".format(
                                self._session))

    def _get_response_from_config(self, response_to_get):
        """ Given a response to get out of the config file, try to get it and
        send it to the robot to play.  If it's not there, print an error.
        """
        if response_to_get in self._script_config:
            resp = self._script_config[response_to_get][random.randint(
                0, len(self._script_config[response_to_get]) - 1)]
            return resp
        else:
            self._logger.warning("No {} found in the script config, so we "
                                 "cannot pick one to play!".format(
                                     response_to_get))

    def set_end_interaction(self):
        """ End the game now -- make the robot fall asleep and say we're done.
        """
        # Tell the robot to stop fidgeting, yawn, and fall asleep.
        self._ros_node.send_tega_command(fidgets=TegaAction.FIDGETS_EMPTY)
        self._send_robot_do("YAWN")
        self._send_robot_do("POSE_SLEEPING")
        # Raise a StopIteration exception, which is how we indicate that we're
        # at the end of a script normally.
        raise StopIteration

    def pause_interaction_timer(self):
        """ Track how much time we spend paused so when we check whether we
        have reached the max game time, we don't include time spent paused.
        """
        self._pause_start_time = datetime.datetime.now()
        # Tell the robot to stop fidgeting, yawn, and fall asleep.
        self._ros_node.send_tega_command(fidgets=TegaAction.FIDGETS_EMPTY)
        self._send_robot_do("YAWN")
        self._send_robot_do("POSE_SLEEPING")

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
            if self._repeating:
                self._repeat_time_paused += \
                    datetime.datetime.now() - self._pause_start_time
        # Reset pause start time.
        self._pause_start_time = None
        # Tell the robot to yawn and wakeup.
        self._send_robot_do("YAWN")
        self._send_robot_do("PERKUP")
        self._ros_node.send_tega_command(fidgets=TegaAction.FIDGETS_PHYSICAL)

    def _done_telling_stories(self):
        """ Check whether we're allowed to tell another story or not. """
        # If we've told the max number of stories, or if we've reached max game
        # time, don't load another story even though we were told to. Instead,
        # play error message from robot saying we have to be done now.
        if self._stories_told >= self._max_stories \
                or ((datetime.datetime.now() - self._start_time)
                    - self._total_time_paused >= self._max_game_time):
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
            return True
        else:
            return False

    def _load_scenes_to_pick(self):
        """ Load scene graphics on the tablet for the user to pick from, which
        we use to select which story to play next.
        """
        # For CREATE stories, get the scenes to show for the user to pick from
        # from the participant config.
        if self._session not in self._pconfig:
            self._logger.warning("No session information for {} listed in the "
                                 "participant config file! Cannot check story"
                                 " type for this session. Skipping STORY line."
                                 .format(self._session))
            self._doing_story = False
            return

        if "story_type" not in self._pconfig[self._session]:
            self._logger.warning("No story type listed in the participant "
                                 "config file for session {}! Cannot load "
                                 "story. Skipping STORY line.".format(
                                  self._session))
            self._doing_story = False
            return

        if "create" in self._pconfig[self._session]["story_type"] and \
                "scenes" in self._pconfig[self._session]:
            self._logger.info("Scenes to pick from are: {}".format(
                self._pconfig[self._session]["scenes"]))
            first = True
            for scene in self._pconfig[self._session]["scenes"]:
                self._logger.info("Loading scene {}...".format(scene))
                toload = {}
                toload["name"] = "sr2-scenes/" + scene + "Scene"
                toload["tag"] = "PlayObject"
                toload["draggable"] = False
                if first:
                    toload["position"] = [-240, 180, 0]
                else:
                    toload["position"] = [240, -180, 0]
                toload["scale"] = [35, 35, 35]
                self._ros_node.send_opal_command("LOAD_OBJECT", json.dumps(
                    toload))
                first = False
            # Log that the scenes were shown.
            self._performance_log.log_scenes_shown(
                    self._pconfig[self._session]["scenes"])

    def _load_next_story_script(self):
        """ Load the script for the next story. """
        if self._session not in self._pconfig:
            self._logger.warning("No session information for {} listed in the "
                                 "participant config file! Cannot load next "
                                 "story. Skipping STORY line.".format(
                                     self._session))
            self._doing_story = False
            return

        if "story_type" not in self._pconfig[self._session]:
            self._logger.warning("No story type listed in the participant "
                                 "config for session {}! Cannot load story. "
                                 "Skipping STORY line.".format(
                                     self._session))
            self._doing_story = False
            return

        # Create a script parser for the filename provided, assuming it is
        # in the story scripts directory.
        self._story_parser = ScriptParser()
        try:
            # Try loading the story script. This will either be for the
            # selected scene for a CREATE story or the storybook for a RETELL.
            #
            # CREATE story:
            # The levels of these stories are determined when putting the name
            # of the story to load into the participant config file, since some
            # stories from the SR2-child corpus do not have levels, but stories
            # from the SR2-robot corpus do.
            if "create" in self._pconfig[self._session]["story_type"]:
                # If there are no stories listed in the config, we don't know
                # what story to load.
                if "stories" not in self._pconfig[self._session]:
                    self._logger.warning("No stories are listed in the "
                                         "participant config for session {}! "
                                         "Cannot load story. Skipping STORY "
                                         "line. {}".format(
                                             self._session,
                                             self._pconfig[self._session]))
                    self._doing_story = False
                    return
                # If no scene was selected, we don't know what story to load.
                if not self._selected_scene:
                    self._logger.warning("No scene was selected to play. "
                                         "Negotiation happened? Can't load "
                                         "story without knowing the scene. "
                                         " Skipping STORY line. {}".format(
                                             self._session,
                                             self._pconfig[self._session]))
                    self._doing_story = False
                    return
                # Load story with the script parser!
                self._story_parser.load_script(
                    self._study_path + self._story_script_path +
                    self._pconfig[self._session]["stories"][
                        self._selected_scene] + ".txt")
                self._logger.info("Loading story \"{}\" in scene {}..."
                                  .format(self._pconfig[self._session][
                                      "stories"][self._selected_scene],
                                       self._selected_scene))
                # Log that the story was played.
                self._performance_log.log_played_story(
                        self._pconfig[self._session]["stories"][
                            self._selected_scene],
                        self._selected_scene,
                        self._pconfig["story_create_level"])

            # RETELL story:
            # The levels of these stories are determined by the number listed
            # in the participant config, which is just appended to the story
            # filename.
            elif "retell" in self._pconfig[self._session]["story_type"]:
                # If there is no story name listed in the config, we don't know
                # what story to load and thus can't load it.
                if "story_name" not in self._pconfig[self._session]:
                    self._logger.warning("No story name listed in the "
                                         "participant config for session {}! "
                                         "Cannot load story. Skipping STORY "
                                         "line. {}".format(
                                             self._session,
                                             self._pconfig[self._session]))
                    self._doing_story = False
                    return
                # Get the story retell level; default to level 1 if none is
                # listed in the participant config file.
                level = 1
                if "story_retell_level" not in self._pconfig:
                    self._logger.warning("No story retell level listed in the "
                                         "participant config! {} Defaulting to"
                                         " level 1.".format(self._pconfig))
                else:
                    level = self._pconfig["story_retell_level"]
                # Load the story with the script parser!
                self._story_parser.load_script(
                    self._study_path + self._story_script_path +
                    self._pconfig[self._session]["story_name"] + str(level) +
                    ".txt")
                self._logger.info("Loading story \"{}\" at level {}...".format(
                        self._pconfig[self._session]["story_name"],
                        self._pconfig["story_retell_level"]))
                # Log that the story was played.
                self._performance_log.log_played_story(
                        self._pconfig[self._session]["story_name"], None,
                        self._pconfig["story_retell_level"])
            else:
                self._logger.warning("Neither \"create\" nor \"retell\" is "
                                     "listed as the story type! Thus we don't"
                                     "have a story to load... Skipping STORY "
                                     "line. The selected scene was \"{}\" and "
                                     "here's the config: {}".format(
                                         self._selected_scene,
                                         self._pconfig[self._session]))
                self._doing_story = False
        except IOError as ioerr:
            self._logger.exception("Script parser could not open story script!"
                                   "Skipping STORY line. {}".format(ioerr))
            self._doing_story = False
            return
        except AttributeError as atterr:
            self._logger.exception("Script parser could not open story "
                                   "script because no script was loaded!"
                                   "Skipping STORY line. {}".format(atterr))
            self._doing_story = False
            return
        except KeyError as keyerr:
            self._logger.exception("Could not find scene \"{}\" in the config!"
                                   " Skipping STORY line. {} {}".format(
                                       self._selected_scene,
                                       keyerr,
                                       self._pconfig[self._session]))
            self._doing_story = False
            return

    def _load_next_story_graphics(self):
        """ Set up the game scene and load scene graphics. """
        if self._session not in self._pconfig:
            self._logger.warning("No session information for {} listed in the "
                                 "participant config file! Cannot load next "
                                 "story graphics. Skipping STORY line.".format(
                                     self._session))
            self._doing_story = False
            return

        if "story_type" not in self._pconfig[self._session]:
            self._logger.warning("No story type listed in the participant "
                                 "config for session {}! Cannot load story "
                                 "graphics. Skipping STORY line.".format(
                                     self._session))
            self._doing_story = False
            return

        # CREATE story:
        # TODO it would be straightforward to add moveable characters. Load as
        # PlayObjects that are draggable. Could make a text file for each scene
        # and do "OPAL LOAD_ALL".
        if "create" in self._pconfig[self._session]["story_type"]:
            self._logger.info("Loading CREATE story on Opal device...")

            # But if no scene was selected, we don't know what story to load.
            if not self._selected_scene:
                self._logger.warning("No scene was selected to play. "
                                     "Negotiation happened? Can't load story "
                                     "without knowing the scene. Skipping "
                                     "STORY line. {}".format(
                                         self._session,
                                         self._pconfig[self._session]))
                self._doing_story = False
                return

            toload = {}
            toload["name"] = "sr2-scenes/" + self._selected_scene + "Scene"
            toload["tag"] = "PlayObject"
            toload["draggable"] = False
            toload["position"] = [0, 0, 0]
            toload["scale"] = [80, 80, 80]
            self._ros_node.send_opal_command("LOAD_OBJECT", json.dumps(toload))

        # RETELL story:
        elif "retell" in self._pconfig[self._session]["story_type"]:
            self._logger.info("Loading RETELL story on Opal device...")
            self._ros_node.send_opal_command(
                "STORY_SELECTION",
                self._pconfig[self._session]["story_name"])
            # Start out with the arrow buttons hidden since it's the robot's
            # turn first. When loaded, stories start on the first page by
            # default.
            self._ros_node.send_opal_command("STORY_HIDE_BUTTONS")
