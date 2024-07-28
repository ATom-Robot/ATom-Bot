Import('RTT_ROOT')
Import('rtconfig')
from building import *

objs = []
cwd     = GetCurrentDir()
list = os.listdir(cwd)
src     = Glob('*.c') + Glob('*.cpp')
CPPPATH = [cwd, str(Dir('#'))]

group = DefineGroup('cpu_usage', src, depend = ['PKG_USING_CPU_USAGE'], CPPPATH = CPPPATH)

Return('group')
