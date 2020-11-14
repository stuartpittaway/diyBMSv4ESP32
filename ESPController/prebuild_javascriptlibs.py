""" Script for DIYBMS """
import requests
import datetime
import subprocess
import os
from os import path
import gzip

Import("env")

folder = "."+os.path.sep+'data_src'+os.path.sep

url = 'https://code.jquery.com/jquery-3.5.1.min.js'
r = requests.get(url, allow_redirects=True)

gzip.open(folder+'jquery.js.gz', 'wb').write(r.content)


url = 'https://raw.githubusercontent.com/apache/incubator-echarts/4.9.0/dist/echarts-en.min.js'
r = requests.get(url, allow_redirects=True)

gzip.open(folder+'echarts.min.js.gz', 'wb').write(r.content)

raise Exception("Libraries updated, now you need to check the SHA256 checksums for default.htm")