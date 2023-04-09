set(COMPONENT_ADD_INCLUDEDIRS
    core
    core/common
    )

file(GLOB SRCS
  core/apriltag.c core/apriltag_quad_thresh.c core/tag16h5.c core/tag25h7.c core/tag25h9.c 
  core/tag36h10.c core/tag36h11.c core/tag36artoolkit.c core/common/g2d.c
  core/common/zarray.c core/common/image_u8x3.c
  core/common/zhash.c core/common/zmaxheap.c core/common/unionfind.c core/common/matd.c 
  core/common/image_u8.c core/common/pnm.c core/common/image_f32.c core/common/image_u32.c 
  core/common/workerpool.c core/common/time_util.c core/common/svd22.c 
  core/common/homography.c core/common/string_util.c core/common/getopt.c
  )

set(COMPONENT_SRCS ${SRCS})

register_component()
