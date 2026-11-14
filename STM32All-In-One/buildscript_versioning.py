""" Script for DIYBMS """
import datetime
import subprocess
import os
from os import path

Import("env")

git_sha=None
git_datetime=None

AreWeInGitHubAction = True if "GITHUB_SHA" in env else False

if (AreWeInGitHubAction):
    git_sha=env["GITHUB_SHA"]
else:
    if (path.exists('..'+os.path.sep+'.git')):
        # Get the latest GIT version header/name
        try:
            git_sha = subprocess.check_output(['git','log','-1','--pretty=format:%H']).decode('utf-8')
        except:
            # Ignore any error, user may not have GIT installed
            git_sha = None

if (path.exists('..'+os.path.sep+'.git')):
    # Date of the commit this is built from, as a UNIX timestamp
    try:
        git_datetime = subprocess.check_output(['git','log','-1','--pretty=format:%at']).decode('utf-8')
    except:
        git_datetime = None

# print(env.Dump())

# These are used by GenerateBinaryFile.py
# If a user doesn't have GIT installed, use fake fffff numbers...
if (git_sha!=None):
    env.Append(git_sha=git_sha)
    env.Append(git_sha_short=git_sha[32:])
else:                       
    env.Append(git_sha="ffffffffffffffffffffffffffffffffffffffff")
    env.Append(git_sha_short="ffffffff")

include_dir = os.path.join(env.get('PROJECT_DIR'), 'include')

if (os.path.exists(include_dir) == False):
    raise Exception("Missing include folder")


with open(os.path.join(include_dir, 'EmbeddedFiles_Defines.h'), 'w') as f:
    f.write("// This is an automatically generated file, any changes will be overwritten on compiliation!\n")
    f.write("// DO NOT CHECK THIS INTO SOURCE CONTROL\n")
    f.write("\n\n#ifndef EmbeddedFiles_Defines_H\n#define EmbeddedFiles_Defines_H\n\n")

    # The abbreviated hash, as a number.  The first eight characters, not the last: git
    # resolves an abbreviation by prefix, so "git show c4be162f" on a suffix finds nothing.
    f.write("static const uint32_t GIT_VERSION_SHORT = 0x")
    if (git_sha!=None):
        f.write(git_sha[0:8])
    else:
        #Default for local compile
        f.write("FFFFFFFF")
    f.write("UL;\n\n")



    # Reported to the controller as the date of the commit this was built from, so that
    # rebuilding the same source does not move it.
    #
    # 255/255 with no git to ask, matching GIT_VERSION_SHORT's 0xFFFFFFFF.  Not the 1 Jan
    # 2000 used elsewhere: that is year 0 week 0, and the controller reads a zero word as
    # "this module does not report a date at all".
    if (git_datetime):
        dt = datetime.datetime.utcfromtimestamp(int(git_datetime))
        commit_year = dt.strftime("%y")
        commit_week = str(int(dt.strftime("%W")))
    else:
        commit_year = "255"
        commit_week = "255"

    f.write("static const uint8_t COMMIT_YEAR_BYTE = ")
    f.write(commit_year)
    f.write(";\n\n")

    f.write("static const uint8_t COMMIT_WEEK_NUMBER_BYTE = ")
    f.write(commit_week)
    f.write(";\n\n")

    f.write("#endif")
