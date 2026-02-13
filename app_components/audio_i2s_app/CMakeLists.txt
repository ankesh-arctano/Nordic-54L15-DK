zephyr_library()

zephyr_library_sources(
    i2s_config.c
)

zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../i2c_app
)


zephyr_library_link_libraries(i2c_app)
zephyr_library_named(audio_i2s_app)
zephyr_library_sources(i2s_config.c)

# Hard dependency on i2c_app target
#add_dependencies(zephyr_interface i2c_app)

# Explicitly pull its include directory
target_include_directories(zephyr_interface INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../i2c_app
)