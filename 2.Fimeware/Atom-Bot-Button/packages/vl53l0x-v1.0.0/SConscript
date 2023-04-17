from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add vl53l0x src files.
if GetDepend('PKG_USING_VL53L0X'):
    src += Glob('src/vl53l0x.c')
    src += Glob('vl53l0x/core/src/*.c')
    src += Glob('vl53l0x/platform/src/*.c')
		
if GetDepend('PKG_USING_VL53L0X_SAMPLE'):
    src += Glob('examples/vl53l0x_sample.c')

# add vl53l0x include path.
path  = [cwd + '/inc',cwd+'/vl53l0x/core/inc',cwd+'/vl53l0x/platform/inc']

# add src and include to group.
group = DefineGroup('vl53l0x', src, depend = ['PKG_USING_VL53L0X'], CPPPATH = path)

Return('group')
