'''

Copy bootloader files from framework-arduinoespressif32 to assist GITHUB action

'''

import sys
import struct
import math
import argparse
import os
from os import path
import shutil
import subprocess

Import("env")


def after_build(source, target, env):
    #print("extract_bootloader from framework-arduinoespressif32")
    # print(env.Dump('FLASH_EXTRA_IMAGES'))
    builddir = env.subst(env.get('BUILD_DIR', ''))

    # Get the 2nd tuple from the list, expanding env variables as we go
    files_to_copy = [env.subst(x[-1])
                     for x in env.get('FLASH_EXTRA_IMAGES', [])]

    # Strip out partitions.bin file its already in the correct location
    files_to_copy = [x for x in files_to_copy if x.endswith(
        'partitions.bin') == False]

    # print(files_to_copy)
    # print(len(files_to_copy))

    for file in files_to_copy:
        # file=env.subst(file)
        print("Copying", file)
        shutil.copy2(file, builddir)


def after_buildfs(source, target, env):
    print("Build complete 4MB ESP32 image")
    #print(env.Dump())
    cmd = [sys.executable, env.get('OBJCOPY', ''), "--chip", "esp32", "merge_bin", "-o",
           "esp32-controller-firmware-complete.bin","--flash_size", "4MB"]

    for x in env.get('FLASH_EXTRA_IMAGES', []):
        cmd.append(x[0])
        cmd.append(x[1])

    cmd.append(env.get('ESP32_APP_OFFSET'))
    cmd.append(env.get('PROGNAME')+".bin")
    # This value should be read from 'PARTITIONS_TABLE_CSV'
    cmd.append("0x1c0000")
    cmd.append(env.get('PROGNAME')+".bin")
    # This value should be read from 'PARTITIONS_TABLE_CSV'
    cmd.append("0x370000")
    cmd.append(env.get('ESP32_FS_IMAGE_NAME')+".bin")

    builddir = env.subst(env.get('BUILD_DIR', ''))

    result = subprocess.run(args=cmd, cwd=builddir,
                            shell=True, check=True, capture_output=True)
    print(result.stdout)


env.AddPostAction("buildprog", after_build)
env.AddPostAction("buildfs", after_buildfs)
