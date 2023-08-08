Import("env")

platform = env.PioPlatform()

#my_flags = env.ParseFlags(env['BUILD_FLAGS'])
#defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}
#print(my_flags)
#print(defines.get("DIYBMSMODULEVERSION"))
#print(str(env["BOARD_MCU"]).lower())


if str(env["BOARD_MCU"]).lower()=="stm32f030k6t6":
    env.Replace(PROGNAME="module_fw_%s_%s" % (env["PIOENV"],env.get("BOARD")))


