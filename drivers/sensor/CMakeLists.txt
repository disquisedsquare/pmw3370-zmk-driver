zephyr_library()
zephyr_library_sources(pmw3370.c)
# Add include path so "pmw3370.h" is found
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

