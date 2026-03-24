from building import *
import os

cwd = GetCurrentDir()

CPPPATH = [
    cwd,
    os.path.join(cwd, 'isotp-c'),
]

src = Glob('*.c') + Glob('isotp-c/*.c')

if GetDepend('PKG_ISOTP_C_EXAMPLES'):
    src += Glob('examples/isotp_examples.c')

group = DefineGroup('isotp-c', src, depend=[''], CPPPATH=CPPPATH)

Return('group')