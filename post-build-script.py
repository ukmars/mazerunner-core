# For more information look here
# https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html
#


Import("env", "projenv")

# access to global build environment
# print(env.Dump())

# access to project build environment (is used source files in "src" folder)
# print(projenv.Dump())

# Show sizes
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "avr-size", "$BUILD_DIR/${PROGNAME}.elf"
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)
# Make listing
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "avr-objdump -d -S ", "$BUILD_DIR/${PROGNAME}.elf",
        " > ", "${PROGNAME}.lst"
    ]), "Generate listing file ${PROGNAME}.lst")
)

# format sources
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "clang-format -i ",
        # " --verbose ",
        "${PROJECT_SRC_DIR}/*.cpp",
        "${PROJECT_SRC_DIR}/*.h",
        "${PROJECT_SRC_DIR}/*.ino",
    ]), "Format sources in ${PROJECT_SRC_DIR}")
)


#
# Dump build environment (for debug purpose)
# print(env.Dump())
#

#
# Change build flags in runtime
#
# env.ProcessUnFlags("-DVECT_TAB_ADDR")
# env.Append(CPPDEFINES=("VECT_TAB_ADDR", 0x123456789))

#
# Upload actions
#

# def before_upload(source, target, env):
#     print("before_upload")
#     # do some actions

#     # call Node.JS or other script
#     env.Execute("node --version")


# def after_upload(source, target, env):
#     print("after_upload")
#     # do some actions

# print("Current build targets", map(str, BUILD_TARGETS))

# env.AddPreAction("upload", before_upload)
# env.AddPostAction("upload", after_upload)

#
# Custom actions when building program/firmware
#

# env.AddPreAction("buildprog", callback...)
# env.AddPostAction("buildprog", callback...)

#
# Custom actions for specific files/objects
#

# env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", [callback1, callback2,...])
# env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", callback...)

# custom action before building SPIFFS image. For example, compress HTML, etc.
# env.AddPreAction("$BUILD_DIR/spiffs.bin", callback...)

# custom action for project's main.cpp
# env.AddPostAction("$BUILD_DIR/src/main.cpp.o", callback...)

# Custom HEX from ELF
# env.AddPostAction(
#     "$BUILD_DIR/${PROGNAME}.elf",
#     env.VerboseAction(" ".join([
#         "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
#         "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.hex"
#     ]), "Building $BUILD_DIR/${PROGNAME}.hex")
# )

