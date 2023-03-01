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

# print(env.Dump())

include_dir = os.path.join(env.get('PROJECT_DIR'), 'include')

if (os.path.exists(include_dir) == False):
    raise Exception("Missing include folder")


with open(os.path.join(include_dir, 'EmbeddedFiles_Defines.h'), 'w') as f:
    f.write("// This is an automatically generated file, any changes will be overwritten on compiliation!\n")
    f.write("// DO NOT CHECK THIS INTO SOURCE CONTROL\n")
    f.write("\n\n#ifndef EmbeddedFiles_Defines_H\n#define EmbeddedFiles_Defines_H\n\n")

    f.write("static const char GIT_VERSION[] = \"")
    if (git_sha != None):
        f.write(git_sha)
    else:
        f.write("LocalCompile")
    f.write("\";\n\n")

    f.write("static const char GIT_VERSION_SHORT[] = \"")
    if (git_sha != None):
        f.write(git_sha[-8:])
    else:
        f.write("LocalCompile")
    f.write("\";\n\n")

    f.write("static const uint16_t GIT_VERSION_B1 = 0x")
    if (git_sha != None):
        f.write(git_sha[32:36])
    else:
        # Default for local compile
        f.write("FFFF")
    f.write(";\n\n")

    f.write("static const uint16_t GIT_VERSION_B2 = 0x")
    if (git_sha != None):
        f.write(git_sha[36:])
    else:
        # Default for local compile
        f.write("FFFF")
    f.write(";\n\n")

    f.write("static const char COMPILE_DATE_TIME[] = \"")
    f.write(datetime.datetime.utcnow().isoformat()[:-3]+'Z')
    f.write("\";\n\n")

    f.write("static const char COMPILE_DATE_TIME_SHORT[] = \"")
    f.write(datetime.datetime.utcnow().strftime("%d %b %Y %H:%M"))
    f.write("\";\n\n")

    f.write("static const uint8_t COMPILE_YEAR_BYTE = ")
    f.write(datetime.datetime.utcnow().strftime("%y"))
    f.write(";\n\n")

    f.write("static const uint8_t COMPILE_WEEK_NUMBER_BYTE = ")
    f.write(str(int(datetime.datetime.utcnow().strftime("%W"))))
    f.write(";\n\n")

    # dt=datetime.datetime.utcnow()
    dt=datetime.datetime.utcnow()

    f.write("const char COMPILE_DATE_TIME_UTC[] = \"")
    f.write(dt.isoformat()[:-3]+'Z')
    f.write("\";\n\n")

    f.write("const uint32_t COMPILE_DATE_TIME_UTC_EPOCH = ")
    f.write(int(time.mktime(dt.timetuple())).__str__())
    f.write("UL;\n\n")

    f.write("#endif")
