idf_component_register(
    SRC_DIRS
        "."
        "src"
        "src/rlottie"

    INCLUDE_DIRS
        "."
        "include"
        "src/rlottie"
                )

target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-D RMAKER_DEMO_PROJECT_NAME=\"${CMAKE_PROJECT_NAME}\"")

spiffs_create_partition_image(storage res/spiffs FLASH_IN_PROJECT)
