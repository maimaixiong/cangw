import os
import subprocess

PREFIX = "arm-none-eabi-"
BUILDER = "DEV"

PROJECT = "cangw"
STARTUP_FILE = "startup_stm32f407xx.s"
#MAIN = "cangw/main.c"
PROJECT_FLAGS = [
  "-mcpu=cortex-m4",
  "-mhard-float",
  "-DSTM32F4",
  "-DSTM32F407xx",
  "-mfpu=fpv4-sp-d16",
  "-fsingle-precision-constant",
  "-Os",
  "-g",
  "-DCANGW",
]


def objcopy(source, target, env, for_signature):
    return '$OBJCOPY -O binary %s %s' % (source[0], target[0])


linkerscript_fn = File("stm32_flash.ld").srcnode().abspath

flags = [
  "-Wall",
  "-Wextra",
  "-Wstrict-prototypes",
  "-Werror",
  "-mlittle-endian",
  "-mthumb",
  "-nostdlib",
  "-fno-builtin",
  f"-T{linkerscript_fn}",
  "-std=gnu11",
] + PROJECT_FLAGS


BUILD_TYPE = "DEBUG"
flags += ["-DALLOW_DEBUG"]

includes = [
  ".",
]

stm32f407_env = Environment(
  ENV=os.environ,
  CC=PREFIX + 'gcc',
  AS=PREFIX + 'gcc',
  OBJCOPY=PREFIX + 'objcopy',
  OBJDUMP=PREFIX + 'objdump',
  ASCOM="$AS $ASFLAGS -o $TARGET -c $SOURCES",
  CFLAGS=flags,
  ASFLAGS=flags,
  LINKFLAGS=flags,
  CPPPATH=includes,
  BUILDERS={
    'Objcopy': Builder(generator=objcopy, suffix='.bin', src_suffix='.elf')
  }
)

startup = stm32f407_env.Object(STARTUP_FILE)

# Bootstub
bootstub_elf = stm32f407_env.Program(f"obj/bootstub.{PROJECT}.elf", [startup] + ["bootstub.c"])
bootstub_bin = stm32f407_env.Objcopy(f"obj/bootstub.{PROJECT}.bin", bootstub_elf)

