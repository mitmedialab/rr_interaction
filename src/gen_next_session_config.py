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


def generate_next_session_config(pid, performance, story_dir, study_config):
    """ Given a participant's past performance, determine which stories they
    should hear in the next session.
    """
    print "\nGenerating config for {}...".format(pid)
    print "Past performance is: {}".format(performance)
    # The participant dictionary contains a dictionary for each session.
    # Each session dictionary contains which robot stories were heard, the
    # level of the stories heard, which story scenes were used for telling
    # stories that session, and the text of the participant's stories.
    # It also contains information about what level stories the participant
    # should get for each kind of story (i.e., retell or create).

    # We need particular session information from the study config file to
    # generate the next session's configuration files for each participant.
    if "sessions" not in study_config:
        print "No session configuration found in the study_config file! We " \
              "need this to continue. Exiting."
        exit(1)

    # We need an initial performance log from the participant to be able to
    # generate a config file for their next session.
    if "session" not in performance:
        print "No prior performance information for {}.".format(pid)
        return

    # The current session is the one after the most recent session number that
    # has been recorded for this participant. The session numbers are probably
    # strings so we convert them to ints before checking.
    session = max([int(k) for k in performance["session"].keys()]) + 1
    # Save as an int.
    session_int = session
    # Make it a string since for the dictionaries it's a string as a key.
    session = str(session)
    print "Next is Session {}".format(session)

    # We will generate a dictionary of configuration options for this
    # participant that we will write to a toml file.
    p_config = {}
    p_config[session] = {}

    ##########################################################################
    # PERSONALIZATION: STORIES.
    ##########################################################################
    # TODO function for all this?
    print "Selecting stories for this participant..."
    # Check the session number. For some sessions, we do the story retell task.
    # If this is a story retell session, which robot story will be told?
    print "Checking session... do we do a story retell this time?"
    if "retell" in study_config["sessions"][session]["story_type"]:
        # If so: Specify which story the robot should tell and its level.
        p_config[session]["story"] = study_config["sessions"][session][
                "story_name"]
        p_config[session]["story_level"] = performance["story_retell_level"]
        print "Yes - Story retell: {} at level {}".format(
                p_config[session]["story"], p_config[session]["story_level"])

    # Sometimes the child creates their own story. In this case, we need to
    # pick which story scenes to offer, and which robot story to tell in that
    # scene.
    elif "create" in study_config["sessions"][session]["story_type"]:
        print "No - no retell this time."
        print "Story type: Create\nLooking for story scenes to use..."

        # We need two story scenes for the participant to pick from next.
        # 1. We take any scenes listed in the config. There may only be scenes
        #    listed for one session, since after that we base our scene picks
        #    on which stories are similar/dissimilar to the child's stories,
        #    and that then determines the scenes used.
        if "scenes" in study_config["sessions"][session]:
            p_config[session]["scenes"] = study_config["sessions"][session][
                    "scenes"]
            print "Found scene: {} from study config".format(
                p_config[session]["scenes"])
        else:
            p_config[session]["scenes"] = []
            print "WARNING: No scenes listed in the config for this session."
        # 2. If there was a negotiation last session, we need to include the
        #    scene that wasn't used last time.
        prev_session = str(session_int - 1)
        if "negotiation" in study_config["sessions"][prev_session] and \
                study_config["sessions"][prev_session]["negotiation"]:
            # Check the participant log to see what scenes were offered last
            # session, and which of them were played. If one wasn't played, we
            # can take it for use this session.
            for scene in performance["session"][prev_session]["scenes_shown"]:
                if scene not in performance["session"][prev_session][
                            "scenes_used"]:
                    p_config[session]["scenes"].append(scene)
                    print "Found scene: {} from prior session {}".format(
                            scene, prev_session)
                    break
        # 3. If we don't have two stories yet, we will look for stories for the
        #    robot to tell based on story similarity to the child's stories,
        #    and determine which scenes those stories are in later.

        # We need to pick which robot story is told. The robot tells either
        # stories from the SR2 robot corpus or child stories from the SR2 child
        # corpus. Which corpus to use is specified in the study session config.
        if "corpus" in study_config["sessions"][session]:
            corpus_name = study_config["sessions"][session]["corpus"]
            story_corpus = study_config["story_corpus"][
                    study_config["sessions"][session]["corpus"]]
            print "Checking corpus {} for stories in these scenes...".format(
                    corpus_name)

        # Pick a story for each scene that might be used this session from the
        # stories available for the selected scenes for that corpus.
        p_config[session]["stories"] = {}
        for scene in p_config[session]["scenes"]:
            (_, p_config[session]["stories"][scene], _) = pick_story(
                    story_corpus, corpus_name, performance, story_dir,
                    scene_to_pick_from=scene,
                    similar=study_config["sessions"][session]["similar"])
        # If there are no scenes listed, we'll need to look at stories from all
        # scenes, and then pick stories, and then update the scene list.
        while len(p_config[session]["scenes"]) < \
                study_config["sessions"][session]["num_scenes"]:
            (_, picked_story, picked_scene) = pick_story(
                    story_corpus, corpus_name, performance, story_dir,
                    scenes_used=p_config[session]["scenes"],
                    similar=study_config["sessions"][session]["similar"])
            # Add the scene and story to our participant config file.
            p_config[session]["stories"][picked_scene] = picked_story
            p_config[session]["scenes"].append(picked_scene)

    ##########################################################################
    # PERSONALIZATION: CATCHPHRASES.
    ##########################################################################
    # TODO Add places where we manually add robot catchphrase entrainment?

    ##########################################################################
    # PERSONALIZATION: SHARED NARRATIVE.
    ##########################################################################
    # TODO Add any participant-specific personalization of phrases?

    ##########################################################################
    # PERSONALIZATION: RELATIONSHIP.
    ##########################################################################
    # TODO Add any relationship-related personalization?

    # TODO do something with p_config
    return p_config


def concat_participant_stories(pid):
    """ Concatenate all of the participant's stories together and return as one
    big blob of text.
    """
    pstory = ""
    for session in pid["session"]:
        for story in pid["session"][session]["story_text"]:
            pstory += story + " "
    return pstory


def get_story_from_file(story_file):
    """ Read in text from a given file. """
    try:
        with open(story_file, mode='r') as fil:
            text = fil.read()
        return text
    except IOError as exc:
        print "ERROR: Problem reading robot story file {}\n{}".format(
                story_file, exc)
        raise


def pick_story(story_corpus, corpus_name, pid, story_dir, scenes_used=None,
               scene_to_pick_from=None, similar=True):
    """ Given a set of stories, compute the similarity between the
    stories and the participant's stories, and choose the next story to tell.
    """
    # Get similarity scores between all stories available in the corpus and the
    # participant's stories.  Instead of doing some complicated score
    # averaging, we just get one similarity score per robot story for the
    # participant:
    #   1. Concatenate all of the participant's stories together.
    #   2. Compare this to each robot story in the story corpus, get a score.
    #   3. Pick top robot story for the participant based on scores.
    #   4. Make sure that top robot story isn't in any of the scenes listed so
    #      that we don't duplicate the scene offerings.
    pstory = concat_participant_stories(pid)
    scores = []
    if scene_to_pick_from:
        print "Looking for stories from {}".format(scene_to_pick_from)
    for scene in story_corpus["scenes"]:
        print "Checking scene: {}".format(scene)
        # If we need to pick a story from a particular scene, skip checking
        # stories from any other scenes.
        if scene_to_pick_from and scene != scene_to_pick_from:
            print "\tNot the scene we need - skipping!"
            continue
        for story in story_corpus["scenes"][scene]["stories"]:
            # Get the robot story text from a text file.
            story_name = corpus_name + "/" + scene + \
                story_corpus["delimiter"] + story
            # If there are levels of the stories in this corpus available, we
            # check what level to use for this participant. The level we should
            # use will be listed in their participant dictionary.
            if "level_names" in story_corpus and \
                    story_corpus["level_names"][0] != "":
                story_name += story_corpus["delimiter"] + \
                        story_corpus["level_names"][story_corpus["levels"][
                            pid["story_create_level"]]]
            print "Checking {}...".format(story_name)
            short_name = story_name
            story_name = story_dir + story_name
            story_text = get_story_from_file(story_name + ".txt")
            # Compute the cosine similarity score between the participant's
            # stories and this story.
            score = text_similarity_tools.get_cosine_similarity(story_text,
                                                                pstory)
            scores.append((score, short_name, scene))
            print "Similarity: {}".format(score)

    # Now we have all the scores for the robot stories in the corpus.
    # Which is the top story? Either the max score (if we want a similar story)
    # or the min score (if we want a different story).
    top_match = max(scores) if similar else min(scores)

    # Did we already pick a story in this scene? If so, get the next highest
    # match.
    if scenes_used:
        while top_match[1] in scenes_used:
            # If we already used this scene, remove the top match from the
            # list, get a new top match, and check again.
            print "Top match: {}.\nNot using because we already have a " \
                "story in this scene!".format(top_match)
            scores.remove(top_match)
            if len(scores) < 1:
                print "ERROR: Did not find a scene to use! Exiting because " \
                        "that's a problem you need to fix."
                exit(1)
            top_match = max(scores) if similar else min(scores)

    # If we get here, we found a story in a scene that hasn't been used yet, so
    # return it as the top matching robot story.
    print "Got top match: {}".format(top_match)
    return top_match


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
    print "Updating {}...".format(log["pid"])

    # If we haven't recorded anything for this participant or session before,
    # make new dictionaries to hold the data.
    if log["pid"] not in performance_data:
        performance_data[log["pid"]] = {}
    if "session" not in performance_data[log["pid"]]:
        performance_data[log["pid"]]["session"] = {}
    if log["session"] not in performance_data[log["pid"]]["session"]:
        performance_data[log["pid"]]["session"][str(log["session"])] = {}
    if "overall" not in performance_data[log["pid"]]:
        performance_data[log["pid"]]["overall"] = {}
    if "scenes_used" not in performance_data[log["pid"]]["overall"]:
        performance_data[log["pid"]]["overall"]["scenes_used"] = []
    if "stories_heard" not in performance_data[log["pid"]]["overall"]:
        performance_data[log["pid"]]["overall"]["stories_heard"] = []
    if "stories_heard_levels" not in performance_data[log["pid"]]["overall"]:
        performance_data[log["pid"]]["overall"]["stories_heard_levels"] = []

    # Update!
    performance_data[log["pid"]]["session"][str(log["session"])][
            "scenes_shown"] = log["scenes_shown"]
    performance_data[log["pid"]]["session"][str(log["session"])][
            "stories_heard"] = log["stories_heard"]
    performance_data[log["pid"]]["session"][str(log["session"])][
            "stories_heard_levels"] = log["stories_heard_levels"]
    performance_data[log["pid"]]["session"][str(log["session"])][
            "scenes_used"] = log["scenes_used"]
    performance_data[log["pid"]]["session"][str(log["session"])][
            "story_text"] = log["story_text"]
    # Also add stories told list overall list of story scenes used for ease
    # of reference later.
    performance_data[log["pid"]]["overall"]["scenes_used"] += \
        log["scenes_used"]
    performance_data[log["pid"]]["overall"]["stories_heard"] += \
        log["stories_heard"]
    performance_data[log["pid"]]["overall"]["stories_heard_levels"] += \
        log["stories_heard_levels"]

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
    print "Updated performance log: {}".format(performance)

    # Save the record of participants' past performance to a new file.
    try:
        outname = get_next_outfile_name(prior_performance, out_dir)
        print "Saving updated performance record to {}...".format(outname)
        with open(outname, "w") as outf:
            toml.dump(performance, outf)
    except TypeError as type_ex:
        print "Error writing to file!\nError: {}".format(type_ex)
    return performance


if __name__ == '__main__':
    # Args are:
    # the performance logs to read from,
    # a directory to save the new config files in,
    # a directory with text files with the story texts,
    # a config file containing participant performance so far
    # a config file for the study containing session info and lists of stories
    # a config file containing participant config so far (if any)
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
    PARSER.add_argument("-s, --sconfig", type=str, nargs=1, dest="study_conf",
                        required=True, help="Toml study session config file.")
    # A toml file containing the overall participant config.
    PARSER.add_argument("-c, --pconfig", type=str, nargs="?",
                        dest="participant_conf", required=False,
                        help="Toml participant session config file.")

    ARGS = PARSER.parse_args()
    print "ARGS are: {}".format(ARGS)

    # Process performance logs.
    LOG = process_performance_logs(ARGS.performance[0], ARGS.log_files,
                                   ARGS.outdir[0])

    # Generate a config file for each participant's next session.
    print "Generating config files for next session!"

    # If an existing participant config file was provided, read it in, and add
    # the newly generated config to it, and write it out as a new file with an
    # incremented number in the filename. If we didn't get an existing config
    # file, make a new one.
    if ARGS.participant_conf:
        CONFIG = read_toml(ARGS.participant_conf)
    else:
        CONFIG = {}

    for participant in LOG:
        pconfig = generate_next_session_config(
            participant,
            LOG[participant],
            ARGS.story_dir[0],
            read_toml(ARGS.study_conf[0]))
        # If we did not get a configuration back, it means the information
        # needed to generate the configuration wasn't in our performance log.
        if not pconfig:
            print "Moving on..."
            continue
        # Add the generated configuration to our overall configuration.
        if participant not in CONFIG:
            CONFIG[participant] = {}
        for key in pconfig.keys():
            if key in CONFIG[participant]:
                print "ERROR key exists: {}. Did you already generate config" \
                    " for this session??".format(key)
            else:
                CONFIG[participant][key] = pconfig[key]

    # Save the participant configs out to a new toml file.
    try:
        OUTNAME = ""
        if ARGS.participant_conf:
            OUTNAME = get_next_outfile_name(ARGS.participant_conf,
                                            ARGS.outdir[0])
        else:
            OUTNAME = ARGS.outdir[0] + "participant_config00.toml"
        print "\nSaving updated participant config to {}!".format(OUTNAME)
        with open(OUTNAME, "w") as outf:
            toml.dump(CONFIG, outf)
    except TypeError as type_ex:
        print "Error writing to file: {}\nError: {}".format(OUTNAME, type_ex)
