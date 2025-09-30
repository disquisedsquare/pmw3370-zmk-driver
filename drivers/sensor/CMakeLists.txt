zephyr_library()
zephyr_library_sources(pmw3370.c)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
zephyr_include_directories(include)

zephyr_library_include_directories(${pmw3370-zmk-driver}/app/include)
zephyr_library_include_directories(${pmw3370-zmk-driver}/app/drivers)


