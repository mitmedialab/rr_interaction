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
from rr_msgs.msg import UserInput  # For user input form response constants.


class YesNoUI(QtGui.QWidget):
    """ Buttons for collecting user input regarding the outcomes of a task in
    which the child has to respond positively (yes) or negatively (no).
    """

    def __init__(self, ros_node):
        """ Make buttons for getting yes, no, timeout input from the user.
        """
        super(YesNoUI, self).__init__()

        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up input form's yes/no buttons...")

        # Get reference to ros node so we can do callbacks to publish messages.
        self.ros_node = ros_node

        # Put buttons in a box.
        self.button_box = QtGui.QGroupBox(self)
        self.button_layout = QtGui.QGridLayout(self.button_box)
        self.button_box.setTitle("Yes/No Input Controls")

        # Create interaction control buttons and add to layout.
        # Store the user's selection so we can send it after it is confirmed.
        self.outcome_selected = None

        # Create negotiation outcome buttons and add to layout.
        self.label = QtGui.QLabel(self.button_box)
        self.label.setText("Select scenario outcome: ")
        self.button_layout.addWidget(self.label, 0, 0)

        # No response / timeout.
        tbutton = QtGui.QPushButton("Timeout / no response",
                                    self.button_box)
        tbutton.clicked.connect(lambda: self.on_outcome_selected(
            UserInput.TIMEOUT))
        self.button_layout.addWidget(tbutton, 1, 0)

        # Yes / positive response.
        ybutton = QtGui.QPushButton("Yes: child will do it", self.button_box)
        ybutton.clicked.connect(lambda: self.on_button_selected(
            UserInput.YES))
        self.button_layout.addWidget(ybutton, 1, 0)

        # No / negative response.
        nbutton = QtGui.QPushButton("No: child won't do it", self.button_box)
        nbutton.clicked.connect(lambda: self.on_button_selected(
            UserInput.NO))
        self.button_layout.addWidget(nbutton, 2, 0)

        # Label for confirming choice.
        self.confirm_label = QtGui.QLabel(self.button_box)
        self.confirm_label.setText("You'll need to confirm your selection.")
        self.confirm_label.setWordWrap(True)
        self.button_layout.addWidget(self.confirm_label, 3, 0, 2, 2)

        # Button for confirming choice.
        self.confirm_button = QtGui.QPushButton("CONFIRM AND SEND",
                                                self.button_box)
        self.confirm_button.clicked.connect(lambda: self.on_confirm_button(
                                            self.outcome_selected))
        self.confirm_button.setEnabled(False)
        self.button_layout.addWidget(self.confirm_button, 4, 0)

    def on_button_selected(self, selected):
        """ When a button is pressed, update the label so the user can check it
        and confirm.
        """
        self.outcome_selected = selected
        self.confirm_label.setText("PLEASE CONFIRM: \nSend {}?".format(
                selected))
        self.confirm_button.setStyleSheet('QPushButton {color: red;}')
        self.confirm_button.setEnabled(True)

    def on_confirm_button(self, outcome_selected):
        """ After the user confirms their selection, send the message. """
        # Send message.
        self.ros_node.send_message(UserInput.YESNO, outcome_selected)
        # Reset label and button.
        self.confirm_label.setText("Selection sent.")
        self.confirm_button.setStyleSheet('QPushButton {color: gray;}')
        self.confirm_button.setEnabled(False)
