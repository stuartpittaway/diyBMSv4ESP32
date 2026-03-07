#!/usr/bin/env python3
"""
Download the latest jQuery minified version and replace web_src/jquery.js
"""

import shutil
import os
import urllib.request
import urllib.error

JQUERY_URL = "https://code.jquery.com/jquery-3.7.1.min.js"
JQUERY_LOCAL = "web_src/jquery.js"

print("Downloading jQuery from", JQUERY_URL)

try:
    urllib.request.urlretrieve(JQUERY_URL, JQUERY_LOCAL)
    print("Successfully downloaded and saved to", JQUERY_LOCAL)
    file_size = os.path.getsize(JQUERY_LOCAL)
    print(f"File size: {file_size} bytes")
except urllib.error.URLError as e:
    print(f"WARNING: Failed to download jQuery: {e}")
    print("Continuing with existing", JQUERY_LOCAL)
except Exception as e:
    print(f"WARNING: Unexpected error downloading jQuery: {e}")
    print("Continuing with existing", JQUERY_LOCAL)
