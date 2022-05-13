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

Import("env")

def after_build(source, target, env):
    #print("extract_bootloader from framework-arduinoespressif32")
    #print(env.Dump('FLASH_EXTRA_IMAGES'))
    builddir=env.subst(env.get('BUILD_DIR',''))

    # Get the 2nd tuple from the list, expanding env variables as we go
    files_to_copy = [env.subst(x[-1]) for x in env.get('FLASH_EXTRA_IMAGES',[])]

    # Strip out partitions.bin file its already in the correct location
    files_to_copy = [x for x in files_to_copy if x.endswith('partitions.bin')==False]

    #print(files_to_copy)
    #print(len(files_to_copy))

    for file in files_to_copy:
        #file=env.subst(file)
        print("Copying",file)
        shutil.copy2(file, builddir)

env.AddPostAction("buildprog", after_build)
