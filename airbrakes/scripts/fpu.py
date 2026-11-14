"""Cube's generic builder does not propagate FPU build_flags to the linker.

Select the same hard-float multilib for linking as for C compilation.
"""
Import("env")
env.AppendUnique(LINKFLAGS=["-mfpu=fpv5-d16", "-mfloat-abi=hard"])
