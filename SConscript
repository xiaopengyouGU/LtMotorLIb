import rtconfig
from building import *

cwd = GetCurrentDir()
src = Glob('*.c')
CPPPATH = [cwd]
    
group = DefineGroup('motor_control', src, depend = [''], CPPPATH = CPPPATH)

Return('group')
