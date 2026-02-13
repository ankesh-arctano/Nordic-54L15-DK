# Create a Zephyr library for this component
zephyr_library()

# Source files
zephyr_library_sources(
    i2c_app.c
)

# Export headers to other components / app
zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

zephyr_library_named(i2c_app)
zephyr_library_sources(i2c_app.c)
