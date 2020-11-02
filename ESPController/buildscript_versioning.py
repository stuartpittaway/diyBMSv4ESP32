""" Script for DIYBMS """
import datetime
import subprocess
import os

Import("env")

# Get the latest GIT version header/name
gitversion = subprocess.check_output(['git','log','--pretty=format:%h_%ad','-1','--date=short']).decode('utf-8')
#print(gitversion)
env.Append(CPPDEFINES=('-D GIT_VERSION=\\\"'+gitversion+'\\\"'))
