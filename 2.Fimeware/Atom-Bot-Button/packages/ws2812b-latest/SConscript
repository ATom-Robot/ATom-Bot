from building import *
import rtconfig

cwd     = GetCurrentDir()
src     = Glob('*.c')
CPPPATH = [cwd]
LOCAL_CCFLAGS = ''

if GetDepend(['WS2812B_USING_EXAMPLE']):
    src += Glob('example/ws2812b_example.c')

group = DefineGroup('ws2812b', src, depend = ['PKG_USING_WS2812B'], CPPPATH = CPPPATH, LOCAL_CCFLAGS = LOCAL_CCFLAGS)

Return('group')
