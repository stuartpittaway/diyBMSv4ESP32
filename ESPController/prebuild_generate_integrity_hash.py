""" prepare_integrity_hash for DIYBMS """
import datetime
import subprocess
import os
from os import path
import struct
import glob
import hashlib
import base64


Import("env")

def prepare_integrity_hash():
    #This routine takes JAVASCRIPT files in the web_src folder and generates INTEGRITY checksums
    #using SHA256

    print('prepare_integrity_hash.py')

    data_dir = os.path.join(env.get('PROJECT_DIR'), 'web_src')
    include_dir = os.path.join(env.get('PROJECT_DIR'), 'include')

    if (os.path.exists(data_dir)==False or os.path.exists(include_dir)==False):
        raise Exception("Missing project folder - data or include folder")
 
    all_files = glob.glob(os.path.join(data_dir, '*.js'))

    with open(os.path.join(include_dir,'EmbeddedFiles_Integrity.h'), 'w') as f:

        f.write("// This is an automatically generated file, any changes will be overwritten on compiliation!\n")
        f.write("\n\n#ifndef EmbeddedFiles_Integrity_H\n#define EmbeddedFiles_Integrity_H\n\n")

        for file in all_files:
            print("Integrity {}".format(file))

            name="file_"+os.path.basename(file).replace(".", "_")

            # Generate SHA1 hash of the file for the ETAG HTTP header     
            sha256sum = hashlib.sha256()
            with open(file, 'rb') as source:
                block = source.read(2**16)
                while len(block) != 0:
                    sha256sum.update(block)
                    block = source.read(2**16)
                integrity='sha256-{}'.format(base64.b64encode(sha256sum.digest()).decode())                    

            f.write("const char* const integrity_{} = \"{}\";\n".format(name,integrity))

        f.write("#endif")

prepare_integrity_hash()
