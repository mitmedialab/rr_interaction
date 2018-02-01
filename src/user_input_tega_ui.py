"""
Jacqueline Kory Westlund
February 2018

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

from PySide import QtGui  # Basic GUI stuff.
import logging  # Log messages.
from r1d1_msgs.msg import TegaAction  # For the Tega animation constants.


class TegaUI(QtGui.QWidget):
    """ Buttons for allowing the user to tell Tega to do a few things remotely.
    """

    def __init__(self, ros_node):
        """ Make buttons for telling Tega to do a few things.
        """
        super(TegaUI, self).__init__()

        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up input form's tega control buttons...")

        # Get reference to ros node so we can do callbacks to publish messages.
        self.ros_node = ros_node

        # Put buttons in a box.
        self.button_box = QtGui.QGroupBox(self)
        self.button_layout = QtGui.QGridLayout(self.button_box)
        self.button_box.setTitle("Tega Controls")

        # Create tega control buttons and add to layout.

        # Label for relaying information.
        self.info_label = QtGui.QLabel(self.button_box)
        self.info_label.setText("Click to send command to Tega.")
        self.button_layout.addWidget(self.info_label, 0, 0, 1, 2)

        # Tega sleep.
        sbutton = QtGui.QPushButton("Sleep!", self.button_box)
        sbutton.clicked.connect(lambda: self.on_button_pressed(
            TegaAction.MOTION_POSE_SLEEPING))
        self.button_layout.addWidget(sbutton, 1, 0)

        # Tega wakeup.
        wbutton = QtGui.QPushButton("Wakeup!", self.button_box)
        wbutton.clicked.connect(lambda: self.on_button_pressed(
            TegaAction.MOTION_PERKUP))
        self.button_layout.addWidget(wbutton, 2, 0)

    def on_button_pressed(self, animation):
        """ Send the message for the specified Tega animation. """
        # Send message.
        self.ros_node.send_tega_animation(animation)
        self.info_label.setText("Command sent.")
