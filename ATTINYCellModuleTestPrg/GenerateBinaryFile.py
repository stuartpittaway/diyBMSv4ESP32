Import("env")


# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-I", "ihex", "$BUILD_DIR/${PROGNAME}.hex","-O","binary", "$BUILD_DIR/${PROGNAME}.avrbin"
    ]), "Building binary file $BUILD_DIR/${PROGNAME}.avrbin")
)
