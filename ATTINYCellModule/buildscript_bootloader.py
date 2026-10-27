""" DIYBMS: merge bootloader + application into one HEX file (only environments V450_BL_*)

- checks that the application starts at 0x0600 and fits into 14848 bytes
- creates  <PROGNAME>_with_bootloader.hex  for the one-time UPDI programming
- target "uploadall": writes this file with avrdude (set the fuses first with "-t fuses")
"""
import os

Import("env")

APP_START = 0x0600
APP_MAX = 0x4000 - APP_START

my_flags = env.ParseFlags(env["BUILD_FLAGS"])
defines = {}
for d in my_flags.get("CPPDEFINES", []):
    if isinstance(d, (list, tuple)):
        defines[d[0]] = d[1] if len(d) > 1 else None
    else:
        defines[d] = None

baud = str(defines.get("DIYBMSBAUD"))

# Place the application behind the bootloader (vector table at 0x0600, IVSEL=0 points there)
env.Append(LINKFLAGS=["-Wl,--section-start=.text=0x%04x" % APP_START])
bootloader_hex = os.path.join(env.get("PROJECT_DIR"), "..", "ATTINYBootloader", "prebuilt",
                              "diybms_bootloader_attiny1624_%s.hex" % baud)


def read_hex_records(path):
    with open(path, "r") as f:
        return [l.strip() for l in f if l.strip()]


def hex_address_range(records):
    lo, hi, base = None, None, 0
    for r in records:
        if not r.startswith(":"):
            continue
        n = int(r[1:3], 16)
        addr = int(r[3:7], 16)
        rtype = int(r[7:9], 16)
        if rtype == 0x04:
            base = int(r[9:13], 16) << 16
        elif rtype == 0x02:
            base = int(r[9:13], 16) << 4
        elif rtype == 0x00 and n > 0:
            a = base + addr
            lo = a if lo is None else min(lo, a)
            hi = a + n if hi is None else max(hi, a + n)
    return lo, hi


def merge(target, source, env):
    app_hex = str(target[0])
    if not os.path.exists(bootloader_hex):
        raise Exception("Bootloader missing: %s  (run 'make' in ../ATTINYBootloader)" % bootloader_hex)

    app = read_hex_records(app_hex)
    bl = read_hex_records(bootloader_hex)

    lo, hi = hex_address_range(app)
    if lo != APP_START:
        raise Exception("Application starts at 0x%04X instead of 0x%04X - linker option missing?" % (lo, APP_START))
    if hi - APP_START > APP_MAX:
        raise Exception("Application too large: %d bytes (max. %d)" % (hi - APP_START, APP_MAX))
    blo, bhi = hex_address_range(bl)
    if blo != 0 or bhi > APP_START:
        raise Exception("Bootloader is not within 0x0000-0x05FF")

    out = os.path.join(os.path.dirname(app_hex), env["PROGNAME"] + "_with_bootloader.hex")
    with open(out, "w") as f:
        for r in bl:
            if r.upper() != ":00000001FF":
                f.write(r + "\n")
        for r in app:
            f.write(r + "\n")
    print("Application %d bytes (free %d), combined file: %s" % (hi - APP_START, APP_MAX - (hi - APP_START), out))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", merge)

env.AddCustomTarget(
    name="uploadall",
    dependencies="$BUILD_DIR/${PROGNAME}.hex",
    actions=["$UPLOADER $UPLOADERFLAGS -U flash:w:$BUILD_DIR/${PROGNAME}_with_bootloader.hex:i"],
    title="Bootloader + firmware (UPDI)",
    description="Writes bootloader and application via UPDI. Once before: pio run -t fuses",
)
