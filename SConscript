import os
from building import *

cwd = GetCurrentDir()

CPPPATH = [
    cwd,
    os.path.join(cwd, 'isotp-c')
]

sources = Glob('*.c') + Glob('isotp-c/*.c')

group = DefineGroup('isotp-c', sources, depend=[''], CPPPATH=CPPPATH)

if GetDepend('PKG_ISOTP_C_EXAMPLES'):
    example_sources = Glob('examples/isotp_examples.c')

    example_group = DefineGroup('isotp-c_example', example_sources, depend=[''], CPPPATH=CPPPATH)

    group += example_group

Return('group')