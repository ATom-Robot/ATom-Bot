set(srcs
    "esp32-hal-timer.c"
    "Talkie.cpp"
    )

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ".")