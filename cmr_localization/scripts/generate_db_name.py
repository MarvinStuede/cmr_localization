#!/usr/bin/env python
import time
import sys
millis = int(round(time.time() * 1000))
sys.stdout.write("~/.ros/rtabmap_test_" + str(millis)+ '.db')
