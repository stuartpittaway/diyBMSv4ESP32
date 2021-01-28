import os
from os import path
import json

Import("env")


def generatejson(target, source, env):
    # print("generatejson")
    # print(target)
    # print(source)
    # print(env.Dump())
    # print(env["PROGNAME"])
    manifestjson = os.path.join(env.get('PROJECT_DIR'), '..', 'ESPController', 'data', 'avr', 'manifest.json')

    data = {}

    if os.path.exists(manifestjson):
        with open(manifestjson, 'r') as json_file:
            data = json.load(json_file)

    #print(data)
    #print(env["git_sha_short"])

    signature=""
    board=str(env["PIOENV"]).upper()

    if 'avrprog' not in data:
        data['avrprog'] = []

    # Add other signatures as needed for different AVR chips    
    if str(env["BOARD_MCU"]).lower()=="attiny841":
        signature="1e9315"

    # Delete entry if it currently exists
    for i in range(len(data['avrprog'])): 
        if data['avrprog'][i]['board'] == board: 
            del data['avrprog'][i] 
            break

    efuse=hex(int(env.GetProjectOption("board_fuses.efuse"), 2)).upper()[2:4]
    hfuse=hex(int(env.GetProjectOption("board_fuses.hfuse"), 2)).upper()[2:4]
    lfuse=hex(int(env.GetProjectOption("board_fuses.lfuse"), 2)).upper()[2:4]

    #Add the new entry
    data['avrprog'].append({'board': board, 'name':  env["PROGNAME"]+'.bin', 'ver': env["git_sha_short"],'mcu':signature,'efuse':efuse,'hfuse':hfuse,'lfuse':lfuse})

    with open(manifestjson, 'w') as outfile:
        json.dump(data, outfile)

# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-I", "ihex", "$BUILD_DIR/${PROGNAME}.hex", "-O", "binary", "$BUILD_DIR/${PROGNAME}.bin"
    ]), "Building binary file $BUILD_DIR/${PROGNAME}.bin")
)

# Create the BIN file directly inside the ESPController project, this later gets wrapped into a LITTLEFS file system
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-I", "ihex", "$BUILD_DIR/${PROGNAME}.hex", "-O", "binary",  "$PROJECT_DIR/../ESPController/data/avr/${PROGNAME}.bin"
    ]), "Building binary file in ESPController project")
)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", generatejson)
