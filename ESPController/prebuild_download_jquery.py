#!/usr/bin/env python3
"""
Ensure web_src/jquery.js contains the pinned jQuery release.

- If the existing file already matches the pinned SHA-384 hash, nothing is
  downloaded or written (no network access, no git churn).
- Otherwise the file is downloaded to a temporary file, verified against the
  pinned hash, and only then atomically moved into place.
- If no verified copy can be obtained, the build is stopped rather than
  embedding an unverified file into the firmware.

To upgrade jQuery, change JQUERY_VERSION and JQUERY_SHA384 together.
The official hashes are published at https://code.jquery.com/
"""

import base64
import hashlib
import os
import sys
import tempfile
import urllib.error
import urllib.request

JQUERY_VERSION = "3.7.1"
JQUERY_SHA384 = "sha384-1H217gwSVyLSIfaLxHbE7dRb3v4mYCKbpQvzx0cegeju1MVsGrX5xXxAvs/HgeFs"
JQUERY_URL = f"https://code.jquery.com/jquery-{JQUERY_VERSION}.min.js"
JQUERY_LOCAL = os.path.join("web_src", "jquery.js")
TIMEOUT_SECONDS = 15


def sri_hash(data: bytes) -> str:
    return "sha384-" + base64.b64encode(hashlib.sha384(data).digest()).decode("ascii")


def local_file_is_current() -> bool:
    try:
        with open(JQUERY_LOCAL, "rb") as f:
            data = f.read()
    except OSError:
        return False
    # Tolerate CRLF conversion from git (core.autocrlf) on Windows checkouts
    return sri_hash(data) == JQUERY_SHA384 or sri_hash(data.replace(b"\r\n", b"\n")) == JQUERY_SHA384


def download_and_verify() -> None:
    print("Downloading jQuery", JQUERY_VERSION, "from", JQUERY_URL)
    with urllib.request.urlopen(JQUERY_URL, timeout=TIMEOUT_SECONDS) as response:
        data = response.read()

    actual = sri_hash(data)
    if actual != JQUERY_SHA384:
        raise ValueError(f"hash mismatch: expected {JQUERY_SHA384}, got {actual}")

    # Write to a temp file in the same directory, then atomically replace
    target_dir = os.path.dirname(os.path.abspath(JQUERY_LOCAL))
    fd, tmp_path = tempfile.mkstemp(prefix=".jquery_", suffix=".tmp", dir=target_dir)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
        os.replace(tmp_path, JQUERY_LOCAL)
    except Exception:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise

    print(f"Verified and saved {JQUERY_LOCAL} ({len(data)} bytes)")


if local_file_is_current():
    print(f"jQuery {JQUERY_VERSION} already present and verified, skipping download")
else:
    try:
        download_and_verify()
    except (urllib.error.URLError, OSError, ValueError) as e:
        # Local file is missing or doesn't match the pinned hash, and we couldn't
        # fetch a verified copy - stop rather than embed an unknown file in firmware
        print(f"ERROR: Failed to obtain verified jQuery {JQUERY_VERSION}: {e}")
        print(f"ERROR: {JQUERY_LOCAL} is missing or does not match {JQUERY_SHA384}")
        print("ERROR: Restore it with 'git checkout -- web_src/jquery.js' or check network access")
        sys.exit(1)
