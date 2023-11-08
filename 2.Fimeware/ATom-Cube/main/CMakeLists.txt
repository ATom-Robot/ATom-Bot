idf_component_register(
    SRC_DIRS
        "."
        "src"
        "src/mpu6050"

    INCLUDE_DIRS
        "."
        "include"
        "src/mpu6050"
                )

target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-D RMAKER_DEMO_PROJECT_NAME=\"${CMAKE_PROJECT_NAME}\"")
