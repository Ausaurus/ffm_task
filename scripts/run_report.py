#!/usr/bin/env python3
import subprocess

path = "/home/i_h8_ros/ffm_ws/src/report/script/report.py"
subprocess.run(['python3', path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
