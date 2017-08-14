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
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import unittest
import mock
import random
from mock import Mock
from rr_script_parser import rr_script_parser

class test_script_parser(unittest.TestCase):

    def setUp(self):
        self.sp = rr_script_parser()


    def test_get_session_script(self):
        # Test different session values.
        self.assertEqual(self.sp.get_session_script(-1), "demo.txt")
        self.assertEqual(self.sp.get_session_script(0), "demo.txt")
        self.assertEqual(self.sp.get_session_script(1), "session-1.txt")
        self.assertEqual(self.sp.get_session_script(2), "session-2.txt")
        self.assertEqual(self.sp.get_session_script(333), "session-333.txt")
        # Test invalid session values.
        with self.assertRaises(TypeError):
            self.sp.get_session_script("hi")
        with self.assertRaises(TypeError):
            self.sp.get_session_script()
        with self.assertRaises(TypeError):
            self.sp.get_session_script(3.14)
        with self.assertRaises(TypeError):
            self.sp.get_session_script(0.5)
        with self.assertRaises(ValueError):
            self.sp.get_session_script(-5)


    @mock.patch("__builtin__.open", create=True, autospec=True)
    def test_load_script(self, mock_open):
        value = random.randint(-1000,1000)
        mock_open.side_effect = [ IOError, value ]

        # Get IOError when we try to load a non-existent script.
        with self.assertRaises(IOError):
            self.sp.load_script("demo.txt")

        # Get file handle when we try to load an existent script.
        self.sp.load_script("demo.txt")
        self.assertEqual(self.sp._fh, value)


    def test_next_line(self):
        # Set up mock iterator for the file handle that returns lines
        # from the file.
        value = str(random.randint(-1000,1000))
        m = Mock()
        self.sp._fh = m
        m.next.side_effect = [
                value,
                AttributeError,
                ValueError,
                StopIteration ]

        # A line is returned.
        self.assertEqual(self.sp.next_line(), value)

        # Check each error type:
        # No script file loaded.
        with self.assertRaises(AttributeError):
            self.sp.next_line()

        # Script file closed.
        with self.assertRaises(ValueError):
            self.sp.next_line()

        # End of script file.
        with self.assertRaises(StopIteration):
            self.sp.next_line()


if __name__ == '__main__':
    unittest.main(verbosity=2)
