file(GLOB_RECURSE GIF_SRCS
    src/gif/*.c
)
set(GIF_INCS
    src/gif
)

file(GLOB_RECURSE APP_SRCS
    src/*.c
    src/*.cpp
    src/cmd/*.c
)
set(APP_INCS
    src
    src/cmd
    include
)

file(GLOB_RECURSE SQUARE_SRCS
    src/squareline_ui/*.c
)
set(SQUARE_INCS
    src/squareline_ui
)

idf_component_register(SRCS "main.cpp" ${GIF_SRCS} ${APP_SRCS} ${SQUARE_SRCS}
                    INCLUDE_DIRS "." ${GIF_INCS} ${APP_INCS} ${SQUARE_INCS})

target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-D RMAKER_DEMO_PROJECT_NAME=\"${CMAKE_PROJECT_NAME}\"")

spiffs_create_partition_image(storage res/spiffs FLASH_IN_PROJECT)
