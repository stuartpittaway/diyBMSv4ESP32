#!/usr/bin/env python3
"""
Download ECharts 5.6.0 from CDN and replace web_src/echarts.min.js
"""

import urllib.request
import urllib.error
import os
import time

ECHARTS_URL = "https://cdnjs.cloudflare.com/ajax/libs/echarts/5.6.0/echarts.common.min.js"
ECHARTS_LOCAL = "web_src/echarts.min.js"

print("Downloading ECharts from", ECHARTS_URL)

try:
    urllib.request.urlretrieve(ECHARTS_URL, ECHARTS_LOCAL)
    print("Successfully downloaded and saved to", ECHARTS_LOCAL)
    file_size = os.path.getsize(ECHARTS_LOCAL)
    print(f"File size: {file_size} bytes")
    time.sleep(0.1)  # Brief delay to ensure file is released
except urllib.error.URLError as e:
    print(f"WARNING: Failed to download ECharts: {e}")
    print("Continuing with existing", ECHARTS_LOCAL)
except Exception as e:
    print(f"WARNING: Unexpected error downloading ECharts: {e}")
    print("Continuing with existing", ECHARTS_LOCAL)
