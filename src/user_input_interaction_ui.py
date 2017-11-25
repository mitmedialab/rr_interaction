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

from PySide import QtGui  # Basic GUI stuff.
import logging  # Log messages.
from rr_msgs.msg import UserInput  # For user input form response constants.


class InteractionUI(QtGui.QWidget):
    """ Buttons for allowing the user to start, stop, pause, and resume the
    interaction remotely.
    """

    def __init__(self, ros_node):
        """ Make buttons for getting stop, pause, resume input from the user.
        """
        super(InteractionUI, self).__init__()

        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up input form's interaction buttons...")

        # Get reference to ros node so we can do callbacks to publish messages.
        self.ros_node = ros_node

        # Put buttons in a box.
        self.interaction_box = QtGui.QGroupBox(self)
        self.button_layout = QtGui.QGridLayout(self.button_box)
        self.button_box.setTitle("Interaction Controls")

        # Create interaction control buttons and add to layout.
        # Store the user's selection so we can send it after it is confirmed.
        self.control_selected = None

        # Start interaction
        sbutton = QtGui.QPushButton("Start interaction", self.button_box)
        sbutton.clicked.connect(lambda: self.on_button_selected(
            UserInput.START))
        self.sbutton.setStyleSheet('QPushButton {color: green;}')
        self.button_layout.addWidget(sbutton, 0, 0)

        # Pause interaction.
        pbutton = QtGui.QPushButton("Pause interaction", self.button_box)
        pbutton.clicked.connect(lambda: self.on_button_selected(
            UserInput.PAUSE))
        self.button_layout.addWidget(pbutton, 1, 0)

        # Resume interaction
        rbutton = QtGui.QPushButton("Resume interaction", self.button_box)
        rbutton.clicked.connect(lambda: self.on_button_selected(
            UserInput.RESUME))
        self.button_layout.addWidget(rbutton, 2, 0)

        # Stop interaction.
        stbutton = QtGui.QPushButton("Stop interaction entirely",
                                     self.button_box)
        stbutton.clicked.connect(lambda: self.on_button_selected(
             UserInput.STOP))
        self.stbutton.setStyleSheet('QPushButton {color: red;}')
        self.button_layout.addWidget(stbutton, 3, 0)

        # Label for confirming choice.
        self.confirm_label = QtGui.QLabel(self.button_box)
        self.confirm_label.setText("You'll need to confirm your selection.")
        self.button_layout.addWidget(self.confirm_label, 4, 0)

        # Button for confirming choice.
        self.confirm_button = QtGui.QPushButton("CONFIRM AND SEND",
                                                self.button_box)
        self.confirm_button.clicked.connect(lambda: self.ros_node.send_message(
                                            self.control_selected))
        self.confirm_button.setEnabled(False)
        self.button_layout.addWidget(self.confirm_button, 5, 0)

    def on_button_selected(self, selected):
        """ When a button is pressed, update the label so the user can check it
        and confirm.
        """
        self.control_selected = selected
        self.confirm_label.setText("PLEASE CONFIRM: You chose \"{}\"?".format(
                selected))
        self.confirm_button.setStyleSheet('QPushButton {color: red;}')
        self.confirm_button.setEnabled(True)

    def on_confirm_button(self):
        """ After the user confirms their selection, send the message. """
        # Send message.
        self.ros_node.send_message(UserInput.INTERACTION_CONTROL,
                                   self.control_selected)
        # Reset label and button.
        self.confirm_label.setText("Selection sent.")
        self.confirm_button.setStyleSheet('QPushButton {color: gray;}')
        self.confirm_button.setEnabled(False)
