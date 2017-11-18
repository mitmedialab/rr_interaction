# Relational robot interaction

The relational robot interaction node was designed to run a fully autonomous
relational robot during a long-term child-robot interaction study. This program
loads and launches the interaction for a given study session, using scripts to
list what the robot should do and what to do in response to stuff that happens
during the interaction.

Included are scripts for personalizing the interaction for individual people
based on logs of their performance from previous session.

The program uses ROS to communicate with a Tega robot, an
[Opal](https://github.com/personal-robots/SAR-opal-base) game (generally run on
a tablet) via a rosbridge\_server websocket connection, as well as several
other nodes.

## Setup and dependencies

### Python

You'll need the following libraries. You can `pip install` them; if you do so,
it's recommended that you use the `--user` flag to only install for your user.

- [toml](https://github.com/uiri/toml) (v0.9.3)
- [text_similarity_tools](https://github.com/mitmedialab/text_phrase_matching)
  (v2.0.0)


### Opal tablet communication

Commands to the [opal tablet](https://github.com/mitmedialab/SAR-opal-base) are
sent over a rosbridge\_server websocket connection. For communication with the
tablet to occur, you need to have the rosbridge\_server running, using the
following command:

`roslaunch rosbridge_server rosbridge_websocket.launch`

You will also need to ensure that the opal tablet's config file lists the IP
address or hostname of the machine running roscore. The [opal
tablet](https://github.com/mitmedialab/SAR-opal-base) documentation explains
how to update the config file (it's simple; you change a line in a text file
and copy it to the tablet).

## Usage

Usage: `python src/rr_interaction_node.py [-h] [-e] [-p, --pconfig
[PARTICIPANT_CONFIG]] [session] [participant]`

This will start the main interaction handler, which orchestrates the
interaction: loads scripts, uses ROS to tell the robot and tablet what to do.
Requires roscore to be running and requires rosbridge\_server for communication
with the Opal tablet (where some interaction content may be shown).

positional arguments:
- `session`: Indicate which session this is so the appropriate scripts can be
  loaded. If none is specified, defaults to running a demo session.
- `participant`: Indicate which participant this is so the appropriate scripts
  can be loaded. If none is specified, defaults to running a demo.

optional arguments:
- `-h, --help`: show this help message and exit
- `-e, --use-entrainer`: Send audio to the entrainer on the way to the robot.
- `-p, --pconfig [PARTICIPANT_CONFIG]`: Optional TOML participant-specific
  config file.  Specify if the interaction should be personalized to
  individuals.


### Configuration

#### Interaction config (config.toml)

The top-level interaction config file is written using
[toml](https://github.com/toml-lang/toml). It should be called `config.toml`
and should be located in the top-level of the project repository. You can see
an example config: `config.example.toml`.

The options you can set in this config file include:

- **study\_path**: The relative path from the config file (located at the top of
  the project respository) to the directory with scripts for the current study
  or project. The default is the `interaction_scripts/demo/` directory. We
  recommend creating a new directory in `interaction_scripts/` for each study
  or project.

- **story\_script\_path**: The relative path from the `study_path` directory to
  the directory containing story scripts. For example, the demo story scripts
  are in a subdirectory of the `interaction_scripts/demo` directory named
  `story_scripts`. If the story scripts are not in a subdirectory, set to the
  empty string. If story and session scripts are in the same directory, this
  may be the same as the `story_script_path` option.

- **session\_script\_path**: The relative path from the `study_path` directory to
  the directory containing session scripts. For example, the demo session
  scripts are in a subdirectory of the `interaction_scripts/demo` directory
  named `session_scripts`. If the session scripts are not in a subdirectory,
  set to the empty string. If story and session scripts are in the same
  directory, this may be the same as the `session:w
  _script_path` option.

- **study\_config**: The relative path from the `study_path` directory to the
  study's config file (containing, e.g., definitions for questions, audio, and
  animations).

- **audio\_base\_dir**: The full path to a directory containing audio files
  (only used if audio is streamed to the robot, e.g., if the audio entrainer is
  used, which by default it is). The full path is necessary because the [audio
  entrainer](https://github.com/mitmedialab/rr_audio_entrainer) expects full
  filepaths when you send it audio (i.e., it expects you to give it a wav file
  it can open). So to accommodate this, you can either specify full filepaths
  in your script (which is tedious and not recommended), or, in this config
  file, specify a directory on your file system where you have put all the wav
  files that will be used. These then get streamed to the robot. But you should
  only do one or the other! Note that if you put full filepaths in your script
  (not recommended), you should leave the `audio_base_dir` option in the config
  file blank (i.e., put an empty string there or delete it entirely from the
  file). In this case, you should also leave `viseme_base_dir` blank and put
  your viseme text files in the same directory as your audio files, because
  currently, we assume that viseme files have the same name as their audio
  files, and just replace the file extension.

- **phoneme\_base\_dir**: The full path to a directory containing audio files
  (only used if audio is streamed to the robot, e.g., if the audio entrainer is
  used, which by default it is).

#### Script config

The script config file is written using
[toml](https://github.com/toml-lang/toml). You can name it whatever you want,
so long as it has the `.toml` file extension and you specify where it is in the
top-level interaction config file. Put this config file in your study directory
(specified in the top-level config file in the filed `study_path`).

This config file contains configuration of the robot's questions and audio,
including sets of possible user responses to questions, robot responses to
those user responses, default robot responses when a user times out or doesn't
respond after a maximum number of tries, and lists of animations that should
get played alongside audio files.

See the example included in the demo project for more details:
`interaction_scripts/demo/demo_config.toml`.

#### Log config

The game uses the Python logging module to direct log output to four places:

1. the console
2. debug log file
3. error log file
4. rosout

The game reads in the logging configuration file `rr\_log\_config.json`, which
is located in the `src/` directory.

The options you can set in this log config file include where log files should
be saved, the general format of each log message, and the log level to record
to each log location. We also list the modules that are logging stuff, since we
can set their individual log levels if we so desire as well as list which
logging handlers will be connected to the module. See the [Python documentation
for more details](https://docs.python.org/2/library/logging.html).

If the game cannot read the log config file, it will default to using the
logging module's default setup, logging messages at level DEBUG to "rr.log".

It is worth mentioning that rospy's log system uses the Python logging module
on the back end. If you want to mess with the configuration of rosout or any
other ROS logging, the default config file is generally located at
"$ROS\_ROOT/../../etc/ros/python\_logging.conf" and follows the general Python
logging configuration conventions. According to the [rospy logging
documentation](http://wiki.ros.org/rospy/Overview/Logging#Advanced:_Override_Logging_Configuration),
you can override the location by setting the `ROS_PYTHON_LOG_CONFIG_FILE`
environment variable. You can also change the ROS log level without messing
with this config file by passing the `log_level` parameter to the
`rospy.init_node()` call made in "ss\_game\_node.py". Currently, the log level
DEBUG is passed in to the init\_node call.

By default, ROS saves a node's log files to `~/.ros/log/` or `$ROS_ROOT/log`.
Note that rosout only gets log messages after the node is fully initialized, so
the ROS rosout log file will likely be missing the initial messages. See the
[rospy logging documentation](http://wiki.ros.org/rospy/Overview/Logging) for
more.

#### Participant config

The participant config file is generated after you run the
`gen_next_session_config.py` script to personalize the interaction for
individual participants/users. It will be a
[toml](https://github.com/toml-lang/toml) file. You can name it whatever you
want (so long as it has the `.toml` file extension) by specifying a name when
you run `gen_next_session_config.py`. By default, it will be called
`participant_config00.toml`. Each time you generate the next session's
configuration, the number in the filename will be incremented.

This configuration file will contain all personalization information, such as
which story scenes will be shown, which stories each participant should hear,
what levels they should be at, and several other things.

For more details, see the Personalization section below.

## ROS messages

### Subscribers

The node subscribes to "/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/TegaState" on the ROS topic "/tega_state".

The node subscribes to std_msgs/String messages on the ROS topic
"rr/audio_entrainer".

The node subscribes to
[asr_google_cloud](https://github.com/mitmedialab/asr_google_cloud/)/AsrResult
messages on the ROS topic "/asr_result".

The node subscribes to
[sar_opal_msgs](https://github.com/mitmedialab/sar_opal_msgs)/OpalAction on
the ROS topic "rr/opal_action".

### Publishers

The node publishes "/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/TegaAction" on the ROS topic "/tega".

The node publishes
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/InteractionState"
messages on the ROS topic "/rr/state".

The node publishes
[asr_google_cloud](https://github.com/mitmedialab/asr_google_cloud/)/AsrCommand
messages on the ROS topic "/asr_command".

The node publishes
[sar_opal_msgs](https://github.com/mitmedialab/sar_opal_msgs)/OpalCommand on
the ROS topic "rr/opal_command".

The node publishes
[rr_msgs](https://github.com/mitmedialab/rr_msgs)/EntrainAudio) messages on the
ROS topic "rr/entrain_audio".


## Interaction scripts

The program will attempt to read interaction scripts from the directory listed
in the config file. For the demo interaction, the interaction scripts are
located in the `interaction_scripts/` directory.

### Script format

By default, the demo session uses the `demo.txt` session script. When you
specify a particular session, the session script named `session-[NUMBER].txt`
will be used. For example, if you are loading session 2, the session script
`session-2.txt` will be used.

Script lines should be tab-delimited. Look at the demo script for an example.

The interaction script lists what happens during an interaction session. It
should list, in order, the actions the program should take. These actions
include the following, which are described in more detail below:

- SET
- ROBOT
- OPAL
- WAIT
- REPEAT
- STORY
- QUESTION

#### SET

`SET` will set configuration options or other constants. Currently, you can set
the following:

- MAX\_INCORRECT\_RESPONSES: The maximum number of incorrect responses the user
  can provide for a question before the game moves on.
- MAX\_GAME\_TIME: The maximum amount of time, in minutes, that the user can
  play the game before it exits.
- MAX\_STORIES: The maximum number of stories to tell in one game session.
- PROMPT\_TIME: The amount of time to wait after asking a question before
  playing a prompt, in seconds.
- NUM\_PROMPTS: How many prompts to give before moving on.

For example, the following commands will set the maximum incorrect responses to
2 and the maximum game time allowed to 10 minutes:

`SET MAX_INCORRECT_RESPONSES    2`

`SET MAX_GAME_TIME 10`

#### ROBOT

`ROBOT` is used to send the robot a speech and/or action command. For DO
actions, list the name of the speech-action set to play. For example:

`ROBOT   DO intro-1`

You can also enable or disable fidgets with the FIDGET action. Specify the
name of the fidget set (EMPTY, SPEECH, PHYSICAL), which correspond to the
names in "/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/TegaAction". For example:

`ROBOT  FIDGET  SPEECH`

The robot commands also include a few special commands that indicate that one
of the robot commands from a special list such as `STORY_INTROS` or
`STORY_CLOSINGS` should be used, such as:

`ROBOT  STORY_INTRO`

#### OPAL

`OPAL` is used to send commands to the Opal device. You can include the full
command on the line, following the format defined in
[sar\_opal\_msgs](https://github.com/personal-robots/sar_opal_msgs
"/sar_opal_msgs") for OpalCommand messages. However, use the command name (not
the number!) on the line -- so you'd write `CLEAR` instead of `6`. You can also
use the command `LOAD_ALL` followed by the name of a text file that contains a
list of objects to load with their associated properties to load a particular
set of objects all at once. Similarly, the command `LOAD_STORY` will load the
graphics for the next story.

For example, the following would send opal commands to clear the game scene and
load the pictures for the next story:

`OPAL    CLEAR`

`OPAL    LOAD_STORY`

#### WAIT

`WAIT` is used to wait for a response from the user via a particular trigger
before proceeding in the game. A timeout is specified so that if no response is
received, script playback will continue after that amount of time. The timeout
should be specified in seconds. For example, the following would wait for a
response to a `START` button press and would timeout after 10 seconds:

`WAIT    START  10`

#### REPEAT

`REPEAT` allows you to specify a script that should be repeated multiple times
before continuing on to the next part of the main session script. You also must
specify how many times the script should be repeated by either providing an
integer or providing the name of a set constant, such as `MAX_STORIES`. The
script will be repeated that many times or until the maximum game time has
elapsed, whichever is reached first.

The following example would repeat the `demo-stories.txt` script for
`MAX_STORIES` iterations or until the maximum game time elapsed:

`REPEAT  MAX_STORIES    demo-stories.txt`

#### STORY

`STORY` indicates that the next story for this session should be played back.
Since which stories are played back depends on the session, participant, and
personalization, this can only be determined at the start of a game session --
not ahead of time. Thus, we use the `STORY` command to indicate that it's time
to play back the next story. This command requires no arguments, as the program
will keep its own list of which stories should be played back next, so a line
will look like this:

`STORY`

That said: Depending on your script, you may want to specify that a new story
should be selected before attempting to load the story or play back the story.
A `STORY` line may optionally take a string argument "SETUP", which indicates
that the next story should be selected:

`STORY  SETUP`

You will then need to use the usual script lines for loading and playing back
the story. See `demo-story.txt` for an example.

#### QUESTION

`QUESTION` is used to enter a particular interaction characterized by the robot
asking a question, waiting for a user response, choosing an appropriate robot
response, and looping until either the desired user response is received or a
maximum number of attempts is reached. This command takes the name of a
question (defined in the question config file) as an argument:

`QUESTION   how-are-you-1`

## Personalization

This repository includes scripts for personalizing the interaction run with the
relational robot interaction node. Essentially, given participants' past
performance, generate config files for each participant for the next session.

The personalization depends on several things:

1. A TOML study session config file as (see
   `interaction_scripts/rr2_study/session_config.toml` for an example).
2. An initial TOML performance log file, which should contain the levels at
   which the robot's stories should be for story retell tasks and
   create-a-story tasks for each participant/user.
3. Any performance log files (generated for each interaction run from
   `src/rr_interaction_node.py`).
4. A directory of stories that the robot can tell. Could have sub-directories
   for particular story corpuses.

Usage: `gen_next_session_config.py [-h] -d, --storydir STORY_DIR -o, --outdir
OUTDIR -p, --performance PERFORMANCE -s, --sconfig STUDY_CONF [-c, --pconfig
[PARTICIPANT_CONF]] log_files [log_files ...]`

positional arguments:
- `log_files`: One or more performance log files to process

optional arguments:
- `-h, --help`: show this help message and exit
- `-d, --storydir STORY_DIR`: Directory of robot story texts
- `-o, --outdir OUTDIR`: Directory to save new config files in
- `-p, --performance PERFORMANCE`: Toml file containing the participant
  performance record so far
- `-s, --sconfig STUDY_CONF`: Toml study session config file.
- `-c, --pconfig PARTICIPANT_CONF`: Toml participant session config file.

Example: ` python src/gen_next_session_config.py -d
"interaction_scripts/rr2_study/stories/" -o "performance/" -p
"performance/rr2_performance_log00.toml" -s
"interaction_scripts/rr2_study/session_config.toml" performance/p00*log-1.toml`

### Story personalization

Stories told by the robot for a story retell task are the same for each
participant, but may be at different levels. Thus, the only personalization for
the story retell task is the story level. This level is specified ahead of time
in the first performance log file.

Stories told by the robot for the create-a-story task may be different for each
participant. These stories are selected based on the following criteria:

1. Does the study session config file specify choosing a story from a
   particular story corpus? For example, the `rr2_study` session config file
   specifies either the SR2 child story corpus or the SR2 robot story corpus.
2. Does the study session config file specify choosing stories from any
   particular story scenes? This may be terminology specific to the `rr2_study`
   corpuses, which have a number of different story scenes, each of which has a
   number of stories that can be paired with it.
3. The cosine similarity of the participant's stories to the robot's story. In
   the study session config file, you can either specifiy selecting the _most_
   similar story (i.e., choosing something the participant might like, or that
   is similar in that it might build rapport) or the _least_ similar story
   (i.e., picking a story that exposes the participant to new information or
   new ideas).

The participant config file generated will list what scenes to show for each
participant for the next session, as well as what stories to use in each scene.

## Testing

We are using python's unittest framework for testing.

Steps:
- Run `python -m unittest discover` from the `src/` directory. This will
  automatically find all files in that directory containing tests, and will run
  all the tests.

## Version notes

This program was developed and tested with:

- Python 2.7.6
- Ubuntu 14.04 LTS 32-bit
- ROS Indigo

Some code and documentation in this repository was copied and modified from
[sar_social_stories](https://github.com/mitmedialab/sar_social_stories) 2.3.5.

## Bugs and issues

Please report all bugs and issues on the [rr_interaction github issues
page](https://github.com/mitmedialab/rr_interaction/issues).
