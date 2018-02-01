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


class NegotiationUI(QtGui.QWidget):
    """ Buttons for collecting user input regarding the outcomes of a
    negotiation task.
    """

    def __init__(self, ros_node):
        """ Make buttons for collecting user input regarding a negotiation. """
        super(NegotiationUI, self).__init__()

        # Set up logger.
        self._logger = logging.getLogger(__name__)
        self._logger.info("Setting up input form's negotiation buttons...")

        # Get reference to ros node so we can do callbacks to publish messages.
        self.ros_node = ros_node

        # Negotiation outcome will be stored before we send it.
        self.negotiation_outcome = None

        # Put buttons in a box.
        self.negotiation_box = QtGui.QGroupBox(self)
        self.negotiation_layout = QtGui.QGridLayout(self.negotiation_box)
        self.negotiation_box.setTitle("Negotiation Outcomes")

        # Create negotiation outcome buttons and add to layout.
        self.label = QtGui.QLabel(self.negotiation_box)
        self.label.setText("Select negotiation outcome: ")
        self.negotiation_layout.addWidget(self.label, 0, 0)

        # Outcomes: no response/timeout, refusal, acquiescence, compromise.

        # No response / timeout.
        tbutton = QtGui.QPushButton("Timeout / no response",
                                    self.negotiation_box)
        tbutton.clicked.connect(lambda: self.on_outcome_selected(
            UserInput.TIMEOUT))
        self.negotiation_layout.addWidget(tbutton, 1, 0)

        # Refusal.
        rbutton = QtGui.QPushButton("Refusal: child wants their choice",
                                    self.negotiation_box)
        rbutton.clicked.connect(lambda: self.on_outcome_selected(
            UserInput.REFUSAL))
        self.negotiation_layout.addWidget(rbutton, 2, 0)

        # Acquiesence.
        abutton = QtGui.QPushButton("Acquiescence: child allows robot's "
                                    "choice", self.negotiation_box)
        abutton.clicked.connect(lambda: self.on_outcome_selected(
            UserInput.ACQUIESCENCE))
        self.negotiation_layout.addWidget(abutton, 3, 0)

        # Compromise - general / not specific.
        cgbutton = QtGui.QPushButton("Compromise: Do both (not specific)",
                                     self.negotiation_box)
        cgbutton.clicked.connect(lambda: self.on_outcome_selected(
             UserInput.COMPROMISE_GENERAL))
        self.negotiation_layout.addWidget(cgbutton, 4, 0)

        # Compromise - do child's choice first.
        ccbutton = QtGui.QPushButton("Compromise: Do child's first",
                                     self.negotiation_box)
        ccbutton.clicked.connect(lambda: self.on_outcome_selected(
             UserInput.COMPROMISE_CHILD))
        self.negotiation_layout.addWidget(ccbutton, 5, 0)

        # Compromise - do robot's choice first.
        crbutton = QtGui.QPushButton("Compromise: Do robot's first",
                                     self.negotiation_box)
        crbutton.clicked.connect(lambda: self.on_outcome_selected(
             UserInput.COMPROMISE_ROBOT))
        self.negotiation_layout.addWidget(crbutton, 6, 0)

        # Label for confirming choice.
        self.confirm_label = QtGui.QLabel(self.negotiation_box)
        self.confirm_label.setText("You'll need to confirm your selection.")
        self.confirm_label.setWordWrap(True)
        self.negotiation_layout.addWidget(self.confirm_label, 7, 0, 2, 2)

        # Button for confirming choice.
        self.confirm_button = QtGui.QPushButton("CONFIRM AND SEND",
                                                self.negotiation_box)
        self.confirm_button.clicked.connect(lambda: self.on_confirm_button(
                                            self.negotiation_outcome))
        self.confirm_button.setEnabled(False)
        self.negotiation_layout.addWidget(self.confirm_button, 9, 0)

    def on_outcome_selected(self, selected):
        """ When the outcome is selected, update the label so the user can
        check it and confirm.
        """
        self.negotiation_outcome = selected
        self.confirm_label.setText("PLEASE CONFIRM: \nSend {}?".format(
                selected))
        self.confirm_button.setStyleSheet('QPushButton {color: red;}')
        self.confirm_button.setEnabled(True)

    def on_confirm_button(self, negotiation_outcome):
        """ After the user confirms their selection, send the message. """
        # Send message.
        self.ros_node.send_message(UserInput.NEGOTIATION,
                                   negotiation_outcome)
        # Reset label and button.
        self.confirm_label.setText("Selection sent.")
        self.confirm_button.setStyleSheet('QPushButton {color: gray;}')
        self.confirm_button.setEnabled(False)
