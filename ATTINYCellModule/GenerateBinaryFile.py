import os
from os import path
import json
from shutil import copyfile

Import("env")


def generatejson(target, source, env):
    # This routine takes the compiled AVR code and copies it into the ESPController project
    # ready for converting into a file system image.
    # It also creates a single JSON file "manifest" with the details of the environment, AVR Fuses etc.

    # print("generatejson")
    # print(target)
    # print(source)
    # print(env.Dump())
    # print(env["PROGNAME"])

    source = os.path.join(env.get('PROJECT_BUILD_DIR'),env.get('PIOENV'), env["PROGNAME"]+'.bin')

    if (not os.path.exists(source)):
        raise Exception("Source file not found")

    avrfolder = os.path.join(env.get('PROJECT_DIR'), '..', 'ESPController', 'data', 'avr')

    if (not os.path.exists(avrfolder)):
        os.mkdir(avrfolder)

    my_flags = env.ParseFlags(env['BUILD_FLAGS'])
    defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}

    # Generate a filename which is less than 35 characters (LittleFS max)
    newfilename="fw_%s_%s.bin" % (env["PIOENV"], env["git_sha_short"] )

    if len(newfilename)>35:
        print (newfilename)
        raise Exception("Generated file name is longer than 35 chars")

    copyfile(source,os.path.join(avrfolder,newfilename))

    manifestjson = os.path.join(avrfolder, 'manifest.json')

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
    # Make sure you lowercase the comparison!
    if str(env["BOARD_MCU"]).lower()=="attiny841":
        signature="1e9315"

    if str(env["BOARD_MCU"]).lower()=="attiny1614":
        signature="1e9422"

    if str(env["BOARD_MCU"]).lower()=="attiny1624":
        signature="1e942a"
        
    if (signature==""):
        raise Exception("Unknown chip signature")

    # Delete entry if it currently exists
    for i in range(len(data['avrprog'])): 
        if data['avrprog'][i]['board'] == board: 
            del data['avrprog'][i] 
            break

    if str(env["BOARD_MCU"]).lower()=="attiny1624":
        #Add the new entry
        data['avrprog'].append({'board': board, 'name':  newfilename, 'ver': env["git_sha_short"],'mcu':signature,'efuse':0,'hfuse':0,'lfuse':0})

    if str(env["BOARD_MCU"]).lower()=="attiny824":
        #Add the new entry
        data['avrprog'].append({'board': board, 'name':  newfilename, 'ver': env["git_sha_short"],'mcu':signature,'efuse':0,'hfuse':0,'lfuse':0})

    if str(env["BOARD_MCU"]).lower()=="attiny841":
        efuse=hex(int(env.GetProjectOption("board_fuses.efuse"), 2)).upper()[2:4]
        hfuse=hex(int(env.GetProjectOption("board_fuses.hfuse"), 2)).upper()[2:4]
        lfuse=hex(int(env.GetProjectOption("board_fuses.lfuse"), 2)).upper()[2:4]

        #Add the new entry
        data['avrprog'].append({'board': board, 'name':  newfilename, 'ver': env["git_sha_short"],'mcu':signature,'efuse':efuse,'hfuse':hfuse,'lfuse':lfuse})

    with open(manifestjson, 'w') as outfile:
        json.dump(data, outfile, indent=4, sort_keys=True)



# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-I", "ihex", "$BUILD_DIR/${PROGNAME}.hex", "-O", "binary", "$BUILD_DIR/${PROGNAME}.bin"
    ]), "Building binary file $BUILD_DIR/${PROGNAME}.bin")
)

# Create the BIN file directly inside the ESPController project, this later gets wrapped into a LITTLEFS file system
#env.AddPostAction(
#    "$BUILD_DIR/${PROGNAME}.hex",
#    env.VerboseAction(" ".join([
#        "$OBJCOPY", "-I", "ihex", "$BUILD_DIR/${PROGNAME}.hex", "-O", "binary",  "$PROJECT_DIR/../ESPController/data/avr/${PROGNAME}.bin"
#    ]), "Building binary file in ESPController project")
#)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", generatejson)
