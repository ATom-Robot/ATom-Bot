from building import *
import rtconfig

# get current directory
cwd     = GetCurrentDir()
# The set of source files associated with this SConscript file.
src     = Glob('src/*.c')
src     += Glob('src/i2c_utils.cpp')
src     += Glob('src/i2c_tools.cpp')

if GetDepend('I2C_TOOLS_USE_SW_I2C'):
	src    += Glob('src/SoftwareI2C.cpp')

path    = [cwd + '/']
path   += [cwd + '/src']

LOCAL_CCFLAGS = ''

group = DefineGroup('i2c-tools', src, depend = ['PKG_USING_I2C_TOOLS'], CPPPATH = path, LOCAL_CCFLAGS = LOCAL_CCFLAGS)

Return('group')
