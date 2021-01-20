Import("env")

# access to global construction environment
#print(env)
# access to project construction environment
#print(projenv)
my_flags = env.ParseFlags(env['BUILD_FLAGS'])
defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}
#print(defines.get("DIYBMSMODULEVERSION"))

env.Replace(PROGNAME="module_firmware_%s_%s%s" % (env["PIOENV"], defines.get("DIYBMSMODULEVERSION") , "_SWAPR19R20" if "SWAPR19R20" in defines else "" ))
