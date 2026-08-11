""" Script for DIYBMS """
import datetime
import time
import subprocess
import os
from os import path

Import("env")

env.Replace(PROGNAME="diybms_controller_firmware_%s_%s" %
            (env["PIOPLATFORM"], env["PIOENV"]))


env.Replace(ESP8266_FS_IMAGE_NAME="diybms_controller_filesystemimage_%s_%s" %
            (env["PIOPLATFORM"], env["PIOENV"]))

env.Replace(ESP32_SPIFFS_IMAGE_NAME="diybms_controller_filesystemimage_%s_%s" %
            (env["PIOPLATFORM"], env["PIOENV"]))

git_sha = None
git_datetime = None

AreWeInGitHubAction = True if "GITHUB_SHA" in env else False

if (AreWeInGitHubAction):
    git_sha = env["GITHUB_SHA"]
else:
    if (path.exists('..'+os.path.sep+'.git')):
        # Get the latest GIT version header/name
        try:
            git_sha = subprocess.check_output(['git', 'log', '-1', '--pretty=format:%H']).decode('utf-8')
        except:
            # Ignore any error, user may not have GIT installed
            git_sha = None

if (path.exists('..'+os.path.sep+'.git')):
    # Date of the commit this is built from, as a UNIX timestamp.  diyBMS-CurrentShunt
    # already reports this one.
    try:
        git_datetime = subprocess.check_output(['git', 'log', '-1', '--pretty=format:%at']).decode('utf-8')
    except:
        git_datetime = None

# print(env.Dump())

include_dir = os.path.join(env.get('PROJECT_DIR'), 'include')

if (os.path.exists(include_dir) == False):
    raise Exception("Missing include folder")


# The commit this was built from, not the moment it was built.  Everyone building the same
# source gets the same number and it only moves forwards, so it can be compared - which a
# truncated commit hash cannot.  Reported over MODBUS as firmwaredatetime, and matches what
# diyBMS-CurrentShunt reports.
#
# Fake date 1 Jan 2000 when there is no git to ask.
epoch = int(git_datetime) if git_datetime else 946684800
dt = datetime.datetime.utcfromtimestamp(epoch)

# The strings live in one translation unit and are declared extern in the header.  Written
# as "static const char x[]" in the header they were emitted once per source file that used
# them - three copies of the commit hash and of the build date.
with open(os.path.join(include_dir, 'EmbeddedFiles_Defines.h'), 'w') as f:
    f.write("// This is an automatically generated file, any changes will be overwritten on compiliation!\n")
    f.write("// DO NOT CHECK THIS INTO SOURCE CONTROL\n")
    f.write("\n\n#ifndef EmbeddedFiles_Defines_H\n#define EmbeddedFiles_Defines_H\n\n")

    f.write("extern const char GIT_VERSION[];\n\n")
    f.write("extern const char GIT_VERSION_SHORT[];\n\n")

    # The first eight characters, not the last: git resolves an abbreviated hash by
    # prefix, so "git show c4be162f" on a suffix finds nothing.
    f.write("static const uint16_t GIT_VERSION_B1 = 0x")
    f.write(git_sha[0:4] if git_sha != None else "FFFF")
    f.write(";\n\n")

    f.write("static const uint16_t GIT_VERSION_B2 = 0x")
    f.write(git_sha[4:8] if git_sha != None else "FFFF")
    f.write(";\n\n")

    f.write("extern const char COMMIT_DATE_TIME[];\n\n")

    # Reported to Victron as the firmware version, so it has to describe the source rather
    # than the moment it was built.
    f.write("static const uint8_t COMMIT_YEAR_BYTE = ")
    f.write(dt.strftime("%y"))
    f.write(";\n\n")

    f.write("static const uint8_t COMMIT_WEEK_NUMBER_BYTE = ")
    f.write(str(int(dt.strftime("%W"))))
    f.write(";\n\n")

    f.write("extern const uint32_t COMMIT_DATE_TIME_UTC_EPOCH;\n\n")

    f.write("#endif")

with open(os.path.join(env.get('PROJECT_DIR'), 'src', 'EmbeddedFiles_Defines.cpp'), 'w') as f:
    f.write("// This is an automatically generated file, any changes will be overwritten on compiliation!\n")
    f.write("// DO NOT CHECK THIS INTO SOURCE CONTROL\n\n")
    f.write("#include <stdint.h>\n\n")

    f.write("extern const char GIT_VERSION[] = \"")
    f.write(git_sha if git_sha != None else "LocalCompile")
    f.write("\";\n\n")

    f.write("extern const char GIT_VERSION_SHORT[] = \"")
    f.write(git_sha[0:8] if git_sha != None else "LocalCompile")
    f.write("\";\n\n")

    f.write("extern const char COMMIT_DATE_TIME[] = \"")
    f.write(dt.isoformat()[:-3]+'Z')
    f.write("\";\n\n")

    f.write("extern const uint32_t COMMIT_DATE_TIME_UTC_EPOCH = ")
    f.write(str(epoch))
    f.write("UL;\n")
