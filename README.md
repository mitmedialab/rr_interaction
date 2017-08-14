# Relational robot interaction

The relational robot interaction node was designed to run a fully autonomous
relational robot during a long-term child-robot interaction study. This program
loads and launches the interaction for a given study session, using scripts to
list what the robot should do and what to do in response to stuff that happens
during the interaction.

The program uses ROS to communicate with a Tega robot, an
[Opal](https://github.com/personal-robots/SAR-opal-base) game (generally run on a tablet) via a rosbridge\_server websocket
connection, as well as several other nodes.

## Setup and dependencies

### Python

`pip install` the following libraries:

- [toml](https://github.com/uiri/toml)

## Usage

TODO

### Configuration

#### Interaction config

TODO

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

## ROS messages

The node subscribes to "/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/TegaState" on the ROS topic "/tega_state".

The node publishes
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/InteractionState"
messages on the ROS topic "/rr/state".

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

- ADD
- SET
- ROBOT
- OPAL
- WAIT
- REPEAT
- STORY
- QUESTION

#### ADD

`ADD` is used to add a list of robot commands that can be used in response to a
particular trigger. Triggers may be actions the user takes, such as selecting a
correct or incorrect response for a question, input from sensors, or a
particular repeating action in the script, such as an introductory comment
before telling a story. `ADD` should list the trigger and the file containing
the list of robot commands. For example, the following command will load the
commands listed in `incorrect.txt` as response options for incorrect actions
taken by the user:

`ADD INCORRECT_RESPONSE incorrect.txt`

Currently, the following lists can be added:

- CORRECT\_RESPONSES: Responses to correct user actions
- INCORRECT\_RESPONSES: Responses to incorrect user actions
- ANSWER\_FEEDBACK: Responses indicating which action was correct (regardless
  of whether the user performed a correct or incorrect action)
- START\_RESPONSES: Responses to the user selecting a "start" button
- NO\_RESPONSES: Responses to the user selecting a "no" button
- STORY\_INTROS: Introductory comment before telling a story
- STORY\_CLOSINGS: Closing comment after telling a story
- TIMEOUT\_CLOSINGS: Responses for when the maximum game time is reached

#### SET

`SET` will set configuration options or other constants. Currently, you can set
the following:

- MAX\_INCORRECT\_RESPONSES: The maximum number of incorrect responses the user
  can provide for a question before the game moves on.
- MAX\_GAME\_TIME: The maximum amount of time, in minutes, that the user can
  play the game before it exits.
- MAX\_STORIES: The maximum number of stories to tell in one game session.

For example, the following commands will set the maximum incorrect responses to
2 and the maximum game time allowed to 10 minutes:

`SET MAX_INCORRECT_RESPONSES    2`

`SET MAX_GAME_TIME 10`

#### ROBOT

`ROBOT` is used to send the robot a speech and/or action command. For DO
actions, list the name of the speech-action set to play. For example:

`ROBOT   DO intro-1`

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

`QUESTION` is used to enter a particular interaction characterized by the robot asking a question, waiting for a user response, choosing an appropriate robot response, and looping until either the desired user response is received or a maximum number of attempts is reached. This command takes the name of a question (defined in the question config file) as an argument:

`QUESTION   how-are-you-1`

### Robot speech-action config

TODO

### Question config

TODO


## Personalization

TODO


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
