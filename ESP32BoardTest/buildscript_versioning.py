""" Script for DIYBMS """
import datetime
import subprocess
import os
from os import path

Import("env")

env.Replace(PROGNAME="diybms_boardtest_%s_%s" %
            (env["PIOPLATFORM"], env["PIOENV"]))

env.Replace(ESP32_SPIFFS_IMAGE_NAME="diybms_boardtest_fsi_%s_%s" %
            (env["PIOPLATFORM"], env["PIOENV"]))

