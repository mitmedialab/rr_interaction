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

from geometry_msgs.msg import Vector3  # Vectors for lookat coordinates.


# Preset lookat vectors that we use when lookats come up in the script.
# Assumes tablet is stageright of the robot.
LOOKAT_ARR = [ "USER", "TABLET", "UP", "DOWN", "LEFT", "RIGHT" ]
LOOKAT = {
    "USER": Vector3(0, 20, 40),
    "TABLET": Vector3(-20, 20, 40),
    "UP": Vector3(0, 40, 40),
    "DOWN": Vector3(0, 10, 40),
    "LEFT": Vector3(-20, 20, 40),
    "RIGHT": Vector3(20, 20, 40)
}
