set(srcs
    "st7789.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
