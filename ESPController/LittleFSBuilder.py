import os

Import ('env')

mklittlefs='x86_64-linux-gnu-mklittlefs'

if os.name == 'nt':
    mklittlefs='x86_64-w64-mklittlefs.exe'

env.Replace (MKSPIFFSTOOL = os.path.join(env.get('PROJECT_DIR'), mklittlefs) )



print('LittleFSBuilder.py')