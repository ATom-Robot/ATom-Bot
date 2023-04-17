from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add mpu6xxx src files.
if GetDepend('PKG_USING_MPU6XXX'):
    src += Glob('src/mpu6xxx.c')

if GetDepend('RT_USING_SENSOR'):
    src += Glob('src/sensor_inven_mpu6xxx.c')

if GetDepend('PKG_USING_MPU6XXX_SAMPLE'):
    src += Glob('samples/mpu6xxx_sample.c')

# add mpu6xxx include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('mpu6xxx', src, depend = ['PKG_USING_MPU6XXX'], CPPPATH = path)

Return('group')
