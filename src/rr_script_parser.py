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
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import logging # log messages

class rr_script_parser():
    """ Determine which session scripts to load, load them, and provide the
    next line in the script file on request.
    """

    def __init__(self):
        """ Initialize script parser manager """
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up script parser...")


    def get_session_script(self, session):
        """ Get scripts for the specified session """
        if not isinstance(session, int):
            raise TypeError("session should be an integer")

        if session < -1:
            raise ValueError("Session number out of range. Should be -1 to "
                 "play the demo or a positive integer to play a particular "
                 "session.")

        if session <= 0:
            # We will use the demo session script if this is a demo
            # session or if the session number doesn't make sense.
            return "demo.txt"

        # This isn't a demo session, so we need to select a script for
        # the specified session.
        else:
            self._logger.info("We assume session scripts are named with the "
                + "pattern \"session-[session_number].txt\", where the session "
                + "number is an integer starting at 1 for session 1. So for "
                + "this session, we will try to load \"session-" + str(session)
                + ".txt\".")
            return "session-" + str(session) + ".txt"


    def load_script(self, script):
        """ Set up to load script """
        # Open script for reading.
        try:
            self._fh = open(script, "r")
        except IOError as e:
            self._logger.exception("Cannot open script: " + str(script))
            # Pass exception up so anyone trying to load a script
            # knows it didn't work.
            raise
        else:
            # Log that we opened a script.
            self._logger.info("Opened " + str(script))


    def next_line(self):
        """ Get the next line in the script """
        # Read and return next line in script file.
        try:
            return self._fh.next()

        # May get attribute error if file handle does not exist because no
        # script was loaded.
        except AttributeError:
            self._logger.exception("Cannot get next line -- no script loaded!")
            raise

        except ValueError:
            self._logger.exception("Cannot get next line -- script file is "
                + "closed!")
            raise

        except StopIteration:
            self._logger.exception("At end of script file!")
            # Close the script file now that we're done.
            self._fh.close()
            # Pass on the stop iteration exception.
            raise

