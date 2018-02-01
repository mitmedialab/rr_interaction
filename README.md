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

You will also need the following ROS nodes:
- [asr_google_cloud](https://github.com/mitmedialab/asr_google_cloud) (v2.0.0)
- [r1d1_msgs](https://github.com/mitmedialab/r1d1_msgs) (v10.0.0)
- [rr_msgs](https://github.com/mitmedialab/rr_msgs) (v7.0.0)
- [sar_opal_msgs](https://github.com/mitmedialab/sar_opal_msgs) (v5.0.0)


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

Usage: `python src/rr_interaction_node.py [-h] [-e] [session] [participant]`

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

### Launch RR2 study

You will need to run two scripts to launch the RR2 experiment study sessions.
The first is a roslaunch file that launches roscore and various other ROS nodes
that are needed for the study interaction to run. Then, you can launch the
interaction itself using `launch_interaction.py`. This script asks the user for
three arguments (experimenter name, participant ID, and session number), which
it validates before starting the interaction. This is useful for ensuring that
the user enters appropriate parameters for the study (e.g., a valid
experimenter name, since some study scripts use the name to play back a
corresponding audio file).

The scripts have been separated into two so that you can re-launch the
interaction itself without having to restart everything else, e.g., if you
switch participants.

Run:

``` sh
$ roslaunch rr_interaction rr_dependencies.launch
```

Then run:

``` sh
$ python launch_interaction.py
```
Then follow the prompts on screen.

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

- **session\_script\_path**: The relative path from the `study_path` directory
  to the directory containing session scripts. For example, the demo session
  scripts are in a subdirectory of the `interaction_scripts/demo` directory
  named `session_scripts`. If the session scripts are not in a subdirectory,
  set to the empty string. If story and session scripts are in the same
  directory, this may be the same as the `session_script_path` option.

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

- **output\_dir**: The relative path from the config file (located at the top
  of the project respository) to the directory where any output or performance
  files generated during the interaction should be saved.

- **pconfig_dir**: The relative path from the config file (located at the top
  of the project repository) to the directory where any TOML
  participant-specific configs file are located. See below about participant
  config files. These files specify if the interaction should be personalized
  to individuals.

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

The game reads in the logging configuration file `rr_log_config.json`, which
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
`rospy.init_node()` call made in "rr_interaction_node.py". Currently, the log
level DEBUG is passed in to the init\_node call.

By default, ROS saves a node's log files to `~/.ros/log/` or `$ROS_ROOT/log`.
Note that rosout only gets log messages after the node is fully initialized, so
the ROS rosout log file will likely be missing the initial messages. See the
[rospy logging documentation](http://wiki.ros.org/rospy/Overview/Logging) for
more.

**For information about performance logs for participants, see the
Personalization section below.**

**Note about seeing log output:** Depending on how you run the interaction, you
may realize that some log output is not being printed to the terminal screen.
To see the full output, you'll need to look at the ROS node log files
(locations mentioned above). You can tail the appropriate file to see the log
output as it happens, for example: `tail -f ~/.ros/log/relational_robot.log`

The reason for this is that, as it happens, if you _import_ the
`src/rr_interaction_node.py` module somewhere else - such as in our nice
`launch_interaction.py` script - then the script will hang unless
`rospy.init_node()` is called from within a function. If `rospy.init_node()` is
called outside a function, as a module-level thing, the code where you import
this module will hang. It's weird, yes; there is one [ros answer that talks
about it a little](https://answers.ros.org/
question/266612/rospy-init_node-inside-imported-file/). Also! If you do this -
putting the `init_node()` call in a function - then a lot of the log output is
no longer printed to the terminal screen. Not sure why, but the solution for
now is to tail the log file where the output is saved instead.

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
located in the `interaction_scripts/demo/` directory.

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
- IF\_RESPONSE
- STATE

You can put comments in scripts. Comments must start with a `#`.

You can tag lines in the scripts. Tags must start with `**` followed by the
tag. Then write the rest of the line as usual. This can be used in order to add
optional lines to the script that can be executed, or not, based on which
condition a participant is in. For example, the following two lines would play
different intro statements for the robot based on which tag a participant has:

`**RR   ROBOT   DO  intro-relational`
`**NR   ROBOT   DO  intro-basic`


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
graphics for the next story. The command `PICK_STORY` will load graphics that
the user can pick from to choose the next story.

For example, the following would send opal commands to clear the game scene,
load pictures for choosing the next story, and then load the pictures for the
next story, respectively:

`OPAL    CLEAR`

`OPAL    LOAD_STORY`

`OPAL    PICK_STORY`

#### WAIT

`WAIT` is used to wait for a response from the user via a particular trigger
before proceeding in the game. A timeout is specified so that if no response is
received, script playback will continue after that amount of time. The timeout
should be specified in seconds.

For example, the following would wait for a user input response from the GUI
form regarding the outcome of a negotiation, and would timeout after 20
seconds:

`WAIT   USER_INPUT  NEGOTIATION    20`

For example, the following would wait for a response to a `START` button press
on an Opal device and would timeout after 10 seconds:

`WAIT    OPAL   START  10`


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

See `interaction_scripts/demo/session_scripts/demo-story.txt` for an example.

#### QUESTION

`QUESTION` is used to enter a particular interaction characterized by the robot
asking a question, waiting for a user response, choosing an appropriate robot
response, and looping until either the desired user response is received or a
maximum number of attempts is reached. This command takes the name of a
question (defined in the question config file) as an argument:

`QUESTION   how-are-you-1`

#### IF\_RESPONSE

`IF_RESPONSE` is used to say that a particular line should only be executed if
the last `QUESTION` received a user response (i.e., did not time out). This is
very similar to tagging a line. This can be useful for asking follow-up
questions or performing particular actions when a user provides information.
These lines should begin with `IF_RESPONSE` followed by a line as usual, e.g.,
another `QUESTION` or a `ROBOT DO` line:

`IF_RESPONSE    QUESTION    what-else-do-you-like`
`IF_RESPONSE    ROBOT   DO  EXCITED`

#### STATE

`STATE` is used to send an InteractionState message to tag different parts of
the interaction. This can be useful if you plan on recording a rosbag of the
whole interaction, since you will be able to look at portions of the rosbag
pertaining to different states that you record with InteractionState mesages.
The state you record can be any kind of string, for example:

`STATE    start interaction`
`STATE    end negotiation task`

### Checking the scripts

To check whether all the audio and corresponding phoneme files mentioned in the
script files exist in the audio directory, run:

``` sh
$ scripts/check_for_audio.py
```

This script uses the information in the main `config.toml` file to decide which
directories to check for scripts and which directories to check for audio and
phoneme files. It will print a list of all missing files to the screen (i.e.,
files referenced in the scripts that don't exist as audio .wav or phoneme .txt
files).

## User Input GUI

The user can provide input to a running interaction script via the UserInput GUI. This GUI must be started separately from the main interaction node, as follows:

```sh
$ python src/user_input_form.py
```

If you are running the RR2 study with the roslaunch file, the user input form
has been included in the roslaunch file.

The form lets you send two different kinds of UserInput messages:

- Negotiation input (e.g., the outcome of a negotiation with the robot)
- Interaction state input (such as telling the interaction to start, stop,
  pause, and resume)

## Personalization

This repository includes scripts for personalizing the interaction run with the
relational robot interaction node. Essentially, given participants' past
performance, generate config files for each participant for the next session.

The personalization depends on several things:

1. **A TOML study session config file.** See
   `interaction_scripts/rr2_study/session_config.toml` for an example. This
   file configures what happens in each study session, e.g., what type of
   stories the robot will tell and which story scenes get shown by default.
2. **A TOML participant config file**, which should contain the levels at which
   the robot's stories should be for story retell tasks and create-a-story
   tasks for each participant/user, as well as the condition each participant
   is in (i.e., "RR" for relational condition, "NR" for not relational), and
   any other participant-specific information that will need to be in future
   participant config files (such as name). See
   `performance/rr2_participant_config00.toml` for an example.
3. **Any performance log files so far,**  which generated for each interaction
   session run from `src/rr_interaction_node.py` and put in the output
   directory specified in the main `config.toml` file.
4. **A directory of stories** that the robot can tell. Could have
   sub-directories for particular story corpuses. Since this interaction
   involves a lot of storytelling, the story corpuses are used to select
   personalized stories that the robot could tell for each participant.

**Note that you manually create the first participant config file.** After the
first interaction session, you will have performance log files, which you can
feed into the following script to generate the next participant config file as
well as update the master performance log file. The latest participant config
file is used whenever you run an interaction session. It contains all
participant-specific stuff that should happen in an interaction session

**Note that you do not manually create performance log files.** They are always
generated during the interaction session. Then, you can use the following
script to create a master performance log file with the log files from all
interaction sessions, as well as the next participant config file. The log
files are used to track performance and to generate later participant config
files, since they have a record of what each participant has done so far.

Usage: `gen_next_session_config.py [-h] -d, --storydir STORY_DIR -o, --outdir
OUTDIR -s, --sconfig STUDY_CONF [-p, --performance [PERFORMANCE]] -c, --pconfig
PARTICIPANT_CONF log_files [log_files ...]`

positional arguments:
- `log_files`: One or more performance log files to process. These are output
  by the rr_interaction_node after each interaction session.

optional arguments:
- `-h, --help`: show this help message and exit
- `-d, --storydir STORY_DIR`: Directory of robot story texts, with
  subdirectories for different story corpuses.
- `-o, --outdir OUTDIR`: Directory to save new config files in. Wherever you
  like; by default a "performance/" directory.
- `-p, --performance PERFORMANCE`: Latest toml file containing the
  participants' session performance record so far. You will not start with
  this. The first one is generated the first time you run this script and give
  it log files from the first interaction session.
- `-s, --sconfig STUDY_CONF`: Toml study session config file. You create this
  file once; it should contain info about what happens each session as well as
  lists of available stories that can be played.
- `-c, --pconfig PARTICIPANT_CONF`: Toml participant session config file. You
  create the first one manually; after that, they will be automatically
  generated.

Example:
```sh
$ python src/gen_next_session_config.py -d "interaction_scripts/rr2_study/stories/" -o "performance/" -c "performance/rr2_participant_config00.toml" -s "interaction_scripts/rr2_study/session_config.toml" performance/example_performance/p00*log-01.toml
```

### Performance log files

A performance log file will be generated from each interaction session. This
toml file will contain the following information about the participant's
performance and activity for that session:

- pid: The participant's ID number (may also be in the filename).
- session: The interaction session number associated with this log (may also be
  in the filename).
- stories_heard: A list of stories the robot told in the session. E.g.,
  `["PenguinSlide"]`.
- stories_heard_levels: A list corresponding to the stories_heard list with the
  level each story was told at (e.g., `[0, 1]`).
- scenes_shown: A list of story scenes shown to the participant in the session.
  E.g., `["Iceberg", "Treemeadow"]`
- scenes_used = A list of story scenes actually used in the session. Sometimes
  more scenes are offered/shown than are actually selected for use. E.g.,
  `["Iceberg"]`.
- story_text: A list of strings, each containing the full text of a story the
  participant told.

These log files are used to personalize later sessions for individual
participants.

As mentioned above, when you run `gen_next_session_config.py`, an aggregate
performance log containing the performance for all participants for all
sessions is created or updated.

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
