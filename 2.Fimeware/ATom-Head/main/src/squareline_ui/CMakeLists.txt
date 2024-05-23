SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen2.c
    screens/ui_Screen3.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_connected_png.c
    images/ui_img_xyz_png.c
    images/ui_img_head_png.c
    images/ui_img_connect_fail_png.c)

add_library(ui ${SOURCES})
