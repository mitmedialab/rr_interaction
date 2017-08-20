#!/usr/bin/env python
#
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

import toml
import os.path
import re
import argparse

def generate_next_session_config(participant, stories, story_dir):
    """ Given a participant's past performance, determine which stories they
    should hear in the next session.
    """
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



def update_performance(log, performance):
    """ Update the performance dictionary with a participant's most recent
    performance log data.
    """
    # The dictionary 'performance' contains a dictionary for each participant.
    # Each participant dictionary contains a dictionary for each session.
    # Each session dictionary contains which stories were heard and which story
    # scenes were used for telling stories that session.
    # TODO and maybe the text of the participant's stories?
    #
    # The participant log contains the participant ID, the session number, and
    # which stories the participant heard and which story scenes they told
    # stories in for the most recent session.
    print "Updating {}...".format(log["pid"])

    # If we haven't recorded anything for this participant or session before,
    # make new dictionaries to hold the data.
    if log["pid"] not in performance:
        performance[log["pid"]] = {}
    if log["session"] not in performance[log["pid"]]:
        performance[log["pid"]][str(log["session"])] = {}

    # Update!
    performance[log["pid"]][str(log["session"])]["stories_heard"] = \
            log["stories_heard"]
    performance[log["pid"]][str(log["session"])]["stories_told"] = \
            log["stories_told"]
    return performance


def read_toml(toml_file):
    """ Open a toml file and return its contents as a dictionary. """
    with open(toml_file, "r") as tf:
        try:
            contents = toml.loads(tf.read())
            return contents
        # We want to exit if we can't read any of the toml files, so we print
        # out information about the file that couldn't be read and re-raise the
        # exception.
        except TypeError as e:
            print "Didn't read file right: {}. Error: {}".format(tf, e)
            raise
        except TomlDecodeError as e:
            print "Bad toml in file: {}. Error: {}".format(tf, e)
            raise


if __name__ == '__main__':
    # Args are:
    # the performance logs to read from,
    # a directory to save the new config files in,
    # a text file with the list of stories,
    # a directory with text files with the story texts,
    # a config file containing participant performance so far
    description = "Given participants' past performance, generate config " + \
            "files for each participant for the next session."
    parser = argparse.ArgumentParser(description=description)
    # A text file with the list of stories.
    parser.add_argument("-s, --stories", type=str, nargs=1, dest="stories",
            required=True, help="Text file containing the list of story names")
    # A directory with text files with the story texts.
    parser.add_argument("-d, --storydir", type=str, nargs=1, dest="story_dir",
            required=True, help="Directory containing story texts")
    # An output directory to save the new config files in.
    parser.add_argument("-o, --outdir", type=str, nargs=1, dest="outdir",
            required=True, help="Directory to save new config files in")
    # A toml file containing participant performance so far.
    parser.add_argument("-p, --performance", type=str, nargs=1, required=True,
            dest="performance", help="Toml file containing the participant " + \
                    "performance record so far")
    # The performance logs to read from.
    parser.add_argument("log_files", type=str, nargs="+",
            help="One or more performance log files to process")

    args = parser.parse_args()

    # Read in past performance config file.
    print "Reading past performance record..."
    performance = read_toml(args.performance[0])

    # Update the record of participants' past performance using the most recent
    # performance logs.
    print "Updating performance record with new data..."
    for lf in args.log_files:
        performance = update_performance(read_toml(lf), performance)

    # Save the record of participants' past performance to a new file.
    try:
        base = os.path.basename(args.performance[0])
        # Get the number from the performance record's filename and increment
        # it so that we can add it to the new file's name, and thus know which
        # is the latest performance record.
        res = re.findall(r'\d+', base)
        num = ""
        #TODO gets study code number instead of number at end -- fix?
        if len(res) > 0:
            num = str(int(res[0]) + 1)
        # Use the same name and file extension as before, just with the
        # incremented number.
        outname = args.outdir[0] + os.path.splitext(base)[0] + num + \
                os.path.splitext(base)[1]
        print "Saving updated performance record to {}...".format(outname)
        with open(outname, "w") as o:
            toml.dump(performance, o)
    except TypeError as e:
        print "Error writing to file: {}\nError: {}".format(outname, e)

    # Generate a config file for each participant's next session.
    print "Generating config files for next session!"
    for participant in performance:
        generate_next_session_config(participant, args.stories, args.story_dir)

