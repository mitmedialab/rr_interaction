"""
Jacqueline Kory Westlund
November 2017

The MIT License (MIT)

Copyright (c) 2017 Personal Robots Group

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

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

import logging  # Log messages.
import os.path  # To check whether files exist.
import re  # For regex.
import toml  # For dumping to file.


class PerformanceLogger(object):
    """ Log participant performance to a file. This is kept separate from the
    actual interaction code on the offchance that we want to save this data to
    a different location or format, e.g., to a database instead of a toml file.
    """

    def __init__(self, participant, session, directory):
        """ Initialize performance logger. """
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up performance logger...")
        # For holding the log filename that we will use.
        self._filename = ""
        # For holding the performance log.
        self._log = {}
        self._set_up_log(participant, session, directory)
        # For dealing with exuberance entrainment.
        self._total_entrained = 0
        self._mean_intensity = []
        self._speaking_rate = []
        self._duration_factor = []

    def _set_up_log(self, participant, session, directory):
        """ Create an initial performance log file in the specified directory
        for the specified participant and session.
        """
        # Create the initial log.
        self._log["pid"] = participant.lower()
        self._log["session"] = session

        # Try opening a log file.
        self._filename = str(participant) + "-log-00.toml"
        if os.path.isfile(directory + self._filename):
            # If it exists, get the next name to use (i.e. increment the number
            # in the filename) so that we never overwrite an existing log file.
            self._logger.info("Log file \"{}\" already exists. Incrementing "
                              "filename...".format(self._filename))
            self._filename = directory + self._get_next_filename(
                    self._filename)
        else:
            self._filename = directory + self._filename

        # Write the initial log to our new log file.
        self._write_log_to_file()
        # Log that we created a log file for this participant.
        self._logger.info("Created performance log: {}".format(self._filename))

    def _rreplace(self, string, substring, new, num_occurrence):
        """ Replace a substring in a string with a new substring, starting at
        the right, for num_occurrences number of occurrences.
        """
        lis = string.rsplit(substring, num_occurrence)
        return new.join(lis)

    def _get_next_filename(self, filename):
        """ Given a previous filename, get the number from the filename and
        increment it so we can add it to the new file's name, and thus know
        that it is the latest performance record for this participant.
        """
        base, ext = os.path.splitext(os.path.basename(filename))

        # Get the number from the performance record's filename and increment
        # it so that we can add it to the new file's name, and thus know which
        # is the latest performance record.
        res = re.findall(r'\d+', base)
        num = ""

        # If we find at least one number, use the last one (because there may
        # be a number earlier on, e.g., as part of a study code).
        if len(res) > 0:
            num = int(res[len(res) - 1])
        # Remove the rightmost instance of that number from the base filename.
        base = self._rreplace(base, str(num), "", 1)

        # Increment the number and append it back to the filename, and add the
        # file extension back on.
        return base + str(num + 1) + ext

    def log_scenes_shown(self, scenes):
        """ Log that a list of scenes were shown to the participant during the
        negotiation/choice portion.
        """
        self._logger.info("Logging that scenes were shown: {}".format(scenes))
        if "scenes_shown" in self._log:
            for scene in scenes:
                self._log["scenes_shown"].append(scene)
        else:
            self._log["scenes_shown"] = scenes
        self._write_log_to_file()

    def log_played_story(self, story_name, scene, story_level):
        """ Log that a story was played in a scene. """
        self._logger.info("Logging that story {} was played at level {} in "
                          "scene {}".format(story_name, story_level, scene))
        # If we already have some stories and scenes listed as played, just
        # append the new one to the lists. Otherwise, create a new list with
        # the newly played story or scene in it.
        if "stories_heard" in self._log:
            self._log["stories_heard"].append(story_name)
        else:
            self._log["stories_heard"] = [story_name]
        if "stories_heard_levels" in self._log:
            self._log["stories_heard_levels"].append(story_level)
        else:
            self._log["stories_heard_levels"] = [story_level]
        if "scenes_used" in self._log and scene:
            self._log["scenes_used"].append(scene)
        else:
            self._log["scenes_used"] = [scene]
        self._write_log_to_file()

    def log_story_text(self, text):
        """ Log the text of a participant's story. This may also be added to
        the log file manually later if the transcript is not immediately
        available.
        """
        self._logger.info("Logging participant's story: {}".format(text))
        if "story_text" in self._log:
            self._log["story_text"].append(text)
        else:
            self._log["story_text"] = [text]
        self._write_log_to_file()

    def log_negotiation_outcome(self, outcome, child_choice):
        """ Log the outcome of a negotiation about which story scenes to do
        this session.
        """
        self._logger.info("Logging child's scene choice ({}) and negotiation "
                          "outcome: {}".format(outcome, child_choice))
        if "negotiation_outcome" in self._log:
            self._log["negotiation_outcome"].append(outcome)
        else:
            self._log["negotiation_outcome"] = [outcome]
        if "child_scene_choice" in self._log:
            self._log["child_scene_choice"].append(child_choice)
        else:
            self._log["child_scene_choice"] = [child_choice]
        self._write_log_to_file()

    def _write_log_to_file(self):
        """ Write the perfromance log out to the log file. """
        try:
            with open(self._filename, "w") as outf:
                toml.dump(self._log, outf)
        except TypeError as type_ex:
            self._logger.error("Error writing to file: {}\nError: {}".format(
                self._filename, type_ex))
        except IOError as ioe:
            self._logger.error("Error writing to file: {}\nError: {}".format(
                self._filename, ioe))

    def log_entrainment(self, mean_intensity, speaking_rate, duration_factor):
        """ Keep a running average of the mean intensity of the user's speech,
        a running average of the user's speaking rate, and a running average of
        the user's speaking rate relative to the robot's (i.e., how much the
        robot's speech had to be adjusted in order to match the user's).
        """
        # Increment the total number of entrained audio files.
        self._total_entrained += 1
        # Update the lists of mean intensity values, speaking rate, and the
        # duration factor. These are used to compute means across the entire
        # session later.
        self._mean_intensity.append(mean_intensity)
        self._speaking_rate.append(speaking_rate)
        self._duration_factor.append(duration_factor)

        # Update the log.
        self._log["total_entrained_audio"] = self._total_entrained
        if "mean_intensity" in self._log:
            self._log["mean_intensity"].append(mean_intensity)
        else:
            self._log["mean_intensity"] = [mean_intensity]
        if "speaking_rate" in self._log:
            self._log["speaking_rate"].append(speaking_rate)
        else:
            self._log["speaking_rate"] = [speaking_rate]
        if "duration_factor" in self._log:
            self._log["duration_factor"].append(duration_factor)
        else:
            self._log["duration_factor"] = [duration_factor]
        self._write_log_to_file()
