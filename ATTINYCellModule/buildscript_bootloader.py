""" DIYBMS: Bootloader + Anwendung zu einer HEX-Datei zusammenfuegen (nur Umgebungen V450_BL_*)

- prueft, dass die Anwendung bei 0x0600 beginnt und in 14848 Byte passt
- erzeugt  <PROGNAME>_mit_bootloader.hex  fuer die einmalige UPDI-Programmierung
- Ziel "uploadall": schreibt diese Datei per avrdude (Fuses vorher mit "-t fuses")
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

# Anwendung hinter den Bootloader legen (Vektortabelle bei 0x0600, IVSEL=0 zeigt dorthin)
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
        raise Exception("Bootloader fehlt: %s  (in ../ATTINYBootloader 'make' ausfuehren)" % bootloader_hex)

    app = read_hex_records(app_hex)
    bl = read_hex_records(bootloader_hex)

    lo, hi = hex_address_range(app)
    if lo != APP_START:
        raise Exception("Anwendung beginnt bei 0x%04X statt 0x%04X - Linkeroption fehlt?" % (lo, APP_START))
    if hi - APP_START > APP_MAX:
        raise Exception("Anwendung zu gross: %d Byte (max. %d)" % (hi - APP_START, APP_MAX))
    blo, bhi = hex_address_range(bl)
    if blo != 0 or bhi > APP_START:
        raise Exception("Bootloader liegt nicht in 0x0000-0x05FF")

    out = os.path.join(os.path.dirname(app_hex), env["PROGNAME"] + "_mit_bootloader.hex")
    with open(out, "w") as f:
        for r in bl:
            if r.upper() != ":00000001FF":
                f.write(r + "\n")
        for r in app:
            f.write(r + "\n")
    print("Anwendung %d Byte (frei %d), kombinierte Datei: %s" % (hi - APP_START, APP_MAX - (hi - APP_START), out))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", merge)

env.AddCustomTarget(
    name="uploadall",
    dependencies="$BUILD_DIR/${PROGNAME}.hex",
    actions=["$UPLOADER $UPLOADERFLAGS -U flash:w:$BUILD_DIR/${PROGNAME}_mit_bootloader.hex:i"],
    title="Bootloader + Firmware (UPDI)",
    description="Schreibt Bootloader und Anwendung per UPDI. Vorher einmal: pio run -t fuses",
)
