#!/usr/bin/env python
"""
Jacqueline Kory Westlund
November 2017

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

import sys  # Exit and argv.
import logging  # Log messages.
import rospy  # ROS
from PySide import QtGui  # Basic GUI stuff.
from user_input_ros import UserFormROS
from user_input_negotiation_ui import NegotiationUI
from user_input_interaction_ui import InteractionUI
from user_input_tega_ui import TegaUI
from user_input_yesno_ui import YesNoUI


class UserInputForm(QtGui.QMainWindow):
    """ User input form for the Relational Robot project. Used to collect input
    from a user for influencing an ongoing interaction.
    """
    # Set up ROS node.
    ros_node = rospy.init_node('rr_user_input_form', anonymous=False)

    def __init__(self):
        """ Initialize interface and logging. """
        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up user input form...")

        # Setup the GUI form interface.
        super(UserInputForm, self).__init__()
        self.setGeometry(200, 50, 650, 450)
        self.setWindowTitle("RR User Input")

        # Create layout.
        self.central_widget = QtGui.QWidget(self)
        self.central_layout = QtGui.QGridLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        # Setup ROS node publishers and subscribers.
        self.ros = UserFormROS(self.ros_node)

        # Add interaction state buttons.
        interaction_ui = InteractionUI(self.ros)
        self.central_layout.addWidget(interaction_ui, 0, 0, 8, 2)

        # Add Tega control buttons.
        tega_ui = TegaUI(self.ros)
        self.central_layout.addWidget(tega_ui, 9, 0, 3, 2)

        # Add negotiation buttons.
        negotiation_ui = NegotiationUI(self.ros)
        self.central_layout.addWidget(negotiation_ui, 0, 2, 11, 2)

        # Add yes/no buttons.
        yesno_ui = YesNoUI(self.ros)
        self.central_layout.addWidget(yesno_ui, 11, 2, 4, 2)


if __name__ == '__main__':
    # Initialize top-level GUI manager.
    APP = QtGui.QApplication(sys.argv)

    # Start the form.
    try:
        WINDOW = UserInputForm()
        WINDOW.show()

    # if roscore isn't running or shuts down unexpectedly
    except rospy.ROSInterruptException:
        print "ROS node shutdown"

    # Enter main loop, then exit.
    sys.exit(APP.exec_())
