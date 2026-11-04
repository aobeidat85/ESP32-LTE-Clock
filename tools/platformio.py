"""Apply the capture SRAM link guards to the consuming PlatformIO application."""
from pathlib import Path
from SCons.Script import DefaultEnvironment, Dir
Import("env")

guard=(Path(Dir('.').srcnode().abspath).parent/'extras/sram_guard.ld').as_posix()
flag='-Wl,-T,'+guard
env.AppendUnique(LINKFLAGS=[flag])
DefaultEnvironment().AppendUnique(LINKFLAGS=[flag])
