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


import toml  # To read in config files.
from toml import TomlDecodeError  # If we have errors reading config files.
import os.path  # For getting basename, generating next perfomane file name.
import re  # regex!
import argparse  # To get command line args.
import text_similarity_tools  # For getting similarity between stories.


def generate_next_session_config(pid, story_dir, study_config):
    """ Given a participant's past performance, determine which stories they
    should hear in the next session.
    """
    print "Finding stories for {}...".format(pid)
    # The participant dictionary contains a dictionary for each session.
    # Each session dictionary contains which stories were heard and which story
    # scenes were used for telling stories that session.
    # TODO and maybe the text of the participant's stories?

    # Using the lists of stories heard and told so far, and the list of stories
    # available, which story should this participant hear next?
    # TODO

    # Using the lists of stories heard and told so far, and the list of stories
    # available, which story scenes should this participant pick from next?
    # TODO

    # TODO Also add places where we manually add robot catchphrase entrainment?


def update_performance(log, performance_data):
    """ Update the performance dictionary with a participant's most recent
    performance log data.
    """
    # The dictionary 'performance' contains a dictionary for each participant.
    # Each participant dictionary contains a dictionary for each session.
    # Each session dictionary contains which stories were heard and which story
    # scenes were used for telling stories that session, and also the text
    # (i.e. the transcript) of the participant's stories.
    #
    # The participant log contains the participant ID, the session number,
    # which stories the participant heard with their levels, which story scenes
    # they told stories in for the most recent session, and the transcript of
    # their most recent stories.
    print "Updating {} with {}...".format(log["pid"], performance_data)

    # If we haven't recorded anything for this participant or session before,
    # make new dictionaries to hold the data.
    if log["pid"] not in performance_data:
        performance_data[log["pid"]] = {}
    if log["session"] not in performance_data[log["pid"]]:
        performance_data[log["pid"]][str(log["session"])] = {}
    if "overall" not in performance_data[log["pid"]]:
        performance_data[log["pid"]]["overall"] = {}
    if "scenes_used" not in performance_data[log["pid"]]["overall"]:
        performance_data[log["pid"]]["overall"]["scenes_used"] = []
    if "stories_heard" not in performance_data[log["pid"]]["overall"]:
        performance_data[log["pid"]]["overall"]["stories_heard"] = []

    # Update!
    performance_data[log["pid"]][str(log["session"])]["scenes_shown"] = \
        log["scenes_shown"]
    performance_data[log["pid"]][str(log["session"])]["stories_heard"] = \
        log["stories_heard"]
    performance_data[log["pid"]][str(log["session"])]["scenes_used"] = \
        log["scenes_used"]
    performance_data[log["pid"]][str(log["session"])]["story_text"] = \
        log["story_text"]
    # Also add stories told list overall list of story scenes used for ease
    # of reference later.
    performance_data[log["pid"]]["overall"]["scenes_used"] += \
        log["scenes_used"]
    performance_data[log["pid"]]["overall"]["stories_heard"] += \
        log["stories_heard"]

    return performance_data


def read_toml(toml_file):
    """ Open a toml file and return its contents as a dictionary. """
    with open(toml_file, "r") as tof:
        try:
            contents = toml.loads(tof.read())
            return contents
        # We want to exit if we can't read any of the toml files, so we print
        # out information about the file that couldn't be read and re-raise the
        # exception.
        except TypeError as type_ex:
            print "Didn't read file right: {}. Error: {}".format(tof, type_ex)
            raise
        except TomlDecodeError as tde:
            print "Bad toml in file: {}. Error: {}".format(tof, tde)
            raise


def rreplace(string, substring, new, num_occurrence):
    """ Replace a substring in a string with a new substring, starting at the
    right, for num_occurrences number of occurrences.
    """
    lis = string.rsplit(substring, num_occurrence)
    return new.join(lis)


def get_next_outfile_name(filename, directory):
    """ Given a previous filename, get the number from the filename and
    increment it so we can add it to the new file's name, and thus know that it
    is the latest performance record for this participant.
    """
    base, ext = os.path.splitext(os.path.basename(filename))

    # Get the number from the performance record's filename and increment it so
    # that we can add it to the new file's name, and thus know which is the
    # latest performance record.
    res = re.findall(r'\d+', base)
    num = ""
    # If we find at least one number, use the last one (because there may be a
    # number earlier on, e.g., as part of a study code).
    if len(res) > 0:
        num = int(res[len(res) - 1])
    # Remove the rightmost instance of that number from the base filename.
    base = rreplace(base, str(num), "", 1)

    # Increment the number and append it back to the filename, and add the file
    # extension back on.
    return directory + base + str(num + 1) + ext


def process_performance_logs(prior_performance, log_files, out_dir):
    """ Process participants' performance logs and generate new config files
    for the next session.
    """
    # Read in past performance config file.
    print "Reading past performance record..."
    performance = read_toml(prior_performance)

    # Update the record of participants' past performance using the most recent
    # performance logs.
    print "Updating performance record with new data..."
    for logf in log_files:
        performance = update_performance(read_toml(logf), performance)

    # Save the record of participants' past performance to a new file.
    try:
        outname = get_next_outfile_name(prior_performance, out_dir)
        print "Saving updated performance record to {}...".format(outname)
        with open(outname, "w") as outf:
            toml.dump(performance, outf)
    except TypeError as type_ex:
        print "Error writing to file: {}\nError: {}".format(outname, type_ex)
    return performance


if __name__ == '__main__':
    # Args are:
    # the performance logs to read from,
    # a directory to save the new config files in,
    # a directory with text files with the story texts,
    # a config file containing participant performance so far
    # a config file for the study containing session info and lists of stories
    PARSER = argparse.ArgumentParser(
        description="""Given participants' past performance, generate config
        files for each participant for the next session.""")
    # A directory with text files with the story texts.
    PARSER.add_argument("-d, --storydir", type=str, nargs=1, dest="story_dir",
                        required=True, help="Directory of robot story texts")
    # An output directory to save the new config files in.
    PARSER.add_argument("-o, --outdir", type=str, nargs=1, dest="outdir",
                        required=True, help="""Directory to save new config
                        files in""")
    # A toml file containing participant performance so far.
    PARSER.add_argument("-p, --performance", type=str, nargs=1, required=True,
                        dest="performance", help="""Toml file containing the
                        participant performance record so far""")
    # The performance logs to read from.
    PARSER.add_argument("log_files", type=str, nargs="+",
                        help="One or more performance log files to process")
    # A toml file containing the overall study session config.
    PARSER.add_argument("-c, --config", type=str, nargs=1, dest="study_config",
                        required=True, help="Toml study session config file.")

    ARGS = PARSER.parse_args()

    # Process performance logs.
    LOG = process_performance_logs(ARGS.performance[0], ARGS.log_files,
                                   ARGS.out_dir)

    # Generate a config file for each participant's next session.
    print "Generating config files for next session!"
    for participant in LOG:
        generate_next_session_config(participant, ARGS.story_dir,
                                     read_toml(ARGS.study_config))
        # TODO perhaps put all configuration dictionaries for all participants
        # into one big toml file organized by PID?
