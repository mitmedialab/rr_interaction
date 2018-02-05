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

import os  # For checking if files and directories exist.
import os.path  # For checking if files and directories exist.
import toml  # For reading our config file.


def check_script(filename):
    """ Check whether all the audio and viseme files mentioned in the script
    exist in the relevant directories.
    """
    try:
        with open(filename) as fileh:
            script = fileh.readlines()
    except IOError as ioe:
        print "Could not read script \"{}\"! Error: {}".format(filename, ioe)
        return

    # We want to parse all ROBOT DO lines, but only if they have lowercase
    # commands (uppercase is for robot animations). Sometimes these lines may
    # be prefixed with a tag (**NR, **RR) or IF_RESPONSE.
    for line in script:
        if "ROBOT\tDO" in line:
            # Parse this line to find the audio to play. Split line on tabs.
            elements = line.strip().split("\t")

            # If it starts with a tag or IF_RESPONSE, remove that so we have a
            # pure ROBOT DO line.
            if ("**" in elements[0] or "IF_RESPONSE" in elements[0]) and \
                    len(elements) > 1:
                del elements[0]

            # Now the line should be "ROBOT DO thing". If the thing is all
            # uppercase, skip because it's an animation.
            if len(elements) != 3 or elements[2].isupper():
                continue

            # Check the audio.
            check_audio_file(elements[2])
            check_viseme_file(elements[2])


def check_script_config(filename):
    """ Check whether all the audio and viseme files mentioned in the script
    config exist in the relevant directories.
    """
    with open(filename) as tof:
        toml_data = toml.load(tof)

    # Check default responses (max_attempt, timeout_prompts, etc).
    defaults = ["max_attempt", "timeout_prompts", "backchannel_prompts",
                "story_intro", "story_closing", "negotiation_timeout_prompts",
                "negotiation_no_response", "negotiation_refusal",
                "negotiation_acquiesce", "negotiation_general",
                "child_story_closing"]

    for response_type in defaults:
        if response_type in toml_data:
            for audio in toml_data[response_type]:
                check_audio_file(audio)
                check_viseme_file(audio)

    # Check backchannel actions.
    if "backchannel_actions" in toml_data:
        for bca in toml_data["backchannel_actions"]:
            if "actions" not in toml_data["backchannel_actions"][bca]:
                print "ERROR: {} has no \"actions\" line!".format(bca)
            else:
                # Check the backchannel actions listed. Actions in uppercase
                # are robot animations/motion, not speech we need to check.
                for action in toml_data["backchannel_actions"][bca]["actions"]:
                    if action.islower():
                        check_audio_file(action)
                        check_viseme_file(action)

    # Check questions.
    if "questions" in toml_data:
        for question in toml_data["questions"]:
            if "question" not in toml_data["questions"][question]:
                print "ERROR: {} has no \"question\" line!".format(question)
            else:
                # Check the question that is asked.
                check_audio_file(toml_data["questions"][question]["question"])
                check_viseme_file(toml_data["questions"][question]["question"])

            # Check any defaults that are overridden.
            for resp_type in defaults:
                if resp_type in toml_data["questions"][question]:
                    for audio in toml_data["questions"][question][resp_type]:
                        check_audio_file(audio)
                        check_viseme_file(audio)

            # Check all robot responses to user input.
            if "user_input" in toml_data["questions"][question]:
                for user_input_option in toml_data["questions"][question][
                        "user_input"]:
                    for response in user_input_option["robot_responses"]:
                        check_audio_file(response)
                        check_viseme_file(response)
            else:
                print "WARNING: {} has no \"user_input\"!".format(question)

            # Check all robot responses to tablet input.
            if "tablet_input" in toml_data["questions"][question]:
                for user_input_option in toml_data["questions"][question][
                        "tablet_input"]:
                    for response in user_input_option["robot_responses"]:
                        check_audio_file(response)
                        check_viseme_file(response)

    # Check audio.
    if "audio" in toml_data:
        for audio in toml_data["audio"]:
            if "animations" not in toml_data["audio"][audio]:
                print "ERROR: {} has no \"animations\" line!".format(audio)
            if "name" not in toml_data["audio"][audio]:
                print "ERROR: {} has no \"name\" line!".format(audio)
            else:
                check_audio_file(toml_data["audio"][audio]["name"])
                check_viseme_file(toml_data["audio"][audio]["name"])


def check_audio_file(filename):
    """ Check whether a given name exists as an audio file in the audio files
    directory.
    """
    if filename == "" or filename == "<experimenter_name>" or \
            filename == "<participant_name>":
        return
    if not os.path.isfile(AUDIO_BASE_DIR + "source/" + filename + ".wav"):
        MISSING_AUDIO.append(filename + ".wav")


def check_viseme_file(filename):
    """ Check whether a given name exists as an audio file in the audio files
    directory.
    """
    if filename == "" or filename == "<experimenter_name>" or \
            filename == "<participant_name>":
        return
    if not os.path.isfile(VISEME_BASE_DIR + "formatted_phonemes/" + filename
                          + ".txt"):
        MISSING_VISEMES.append(filename + ".txt")


def get_directories():
    """ Read in the main toml config file. """
    # We don't actually have too many branches here; we need to try reading
    # each value from the config file separately.
    # pylint: disable=too-many-branches
    try:
        with open("config.toml") as tof:
            toml_data = toml.load(tof)
        print "Reading config file...: {}".format(toml_data)
        # Directory with scripts for this study.
        if "study_path" in toml_data:
            study_path = toml_data["study_path"]
        else:
            print("Could not read path to interaction "
                  "scripts! Expected option \"study_path\" to"
                  "be in the config file. Exiting because we "
                  "need the scripts to check them.")
            exit(1)
        # Study script config file location.
        if "script_config" in toml_data:
            script_config = toml_data["script_config"]
        else:
            print("Could not read name of script_config! "
                  "Expected option \"script_config\" to be"
                  " in config file. Exiting because we need to check it.")
            exit(1)
        # Directory of story scripts.
        if "story_script_path" in toml_data:
            story_script_path = toml_data["story_script_path"]
        else:
            print("Could not read path to story scripts! "
                  "Expected option \"story_script_path\" to "
                  "be in config file. Exiting because we need to check them.")
            exit(1)
            story_script_path = None
        # Directory of session scripts.
        if "session_script_path" in toml_data:
            session_script_path = toml_data["session_script_path"]
        else:
            print("Could not read path to session scripts! "
                  "Expected option \"session_script_path\" to"
                  " be in config file. Exiting because we need to check them.")
            exit(1)
            session_script_path = None
        # Directory of audio files.
        if "audio_base_dir" in toml_data:
            audio_base_dir = toml_data["audio_base_dir"]
        else:
            print("Could not read audio base directory path! "
                  "Expected option \"audio_base_dir\" to be "
                  "in config file. Assuming audio files are "
                  "in the main study directory.")
            audio_base_dir = None
        # Directory of viseme files.
        if "viseme_base_dir" in toml_data:
            viseme_base_dir = toml_data["viseme_base_dir"]
        else:
            print("Could not read viseme base directory path!"
                  " Expected option \"viseme_base_dir\" to be"
                  " in config file.Assuming audio files are "
                  "in the main study directory.")
            viseme_base_dir = None
    except Exception as exc:  # pylint: disable=broad-except
        print("Could not read your toml config file \""
              "config.toml\". Does the file exist? Is "
              "it valid toml? Exiting because we need the"
              " config file to continue. {}".format(exc))
        exit(1)

    return script_config, study_path + story_script_path, \
        study_path + session_script_path, audio_base_dir, viseme_base_dir


if __name__ == '__main__':
    """ For each audio file named in the toml script files (script_config, and
    each text file in the session scripts and story scripts directories), check
    that it exists in the audio directory named in the overall config file.
    """
    SCRIPT_CONFIG, STORIES_TO_CHECK, SESSIONS_TO_CHECK, AUDIO_BASE_DIR, \
        VISEME_BASE_DIR = get_directories()

    MISSING_AUDIO = []
    MISSING_VISEMES = []

    print "Checking script config..."
    check_script_config(SCRIPT_CONFIG)

    print "Checking story script files..."
    for filename in os.listdir(STORIES_TO_CHECK):
        if filename.endswith(".txt"):
            check_script(STORIES_TO_CHECK + filename)

    print "Checking session script files..."
    for filename in os.listdir(SESSIONS_TO_CHECK):
        if filename.endswith(".txt"):
            check_script(SESSIONS_TO_CHECK + filename)

    # Print all thet missing files, without duplicates.
    print "=================\nMISSING AUDIO FILES:"
    for audio in sorted(list(set(MISSING_AUDIO))):
        print audio

    print "=================\nMISSING VISEME FILES:"
    for viseme in sorted(list(set(MISSING_VISEMES))):
        print viseme
