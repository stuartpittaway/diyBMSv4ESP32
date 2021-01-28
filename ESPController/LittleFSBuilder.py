import os
import stat

Import ('env')

# Override the SPIFF tool with mklittlefs, ideally this script would download
# the relevant version depending on operating system automatically, but
# for now they are embedded in the project

mklittlefs='x86_64-linux-gnu-mklittlefs'

if os.name == 'nt':
    mklittlefs='x86_64-w64-mklittlefs.exe'
else:
    st = os.stat(mklittlefs)
    os.chmod(mklittlefs, st.st_mode | stat.S_IEXEC)

env.Replace (MKSPIFFSTOOL = os.path.join(env.get('PROJECT_DIR'), mklittlefs) )

print('LittleFSBuilder.py using '+mklittlefs)