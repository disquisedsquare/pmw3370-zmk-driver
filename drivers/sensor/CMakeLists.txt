zephyr_library()
zephyr_library_sources(pmw3370.c)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
zephyr_include_directories(include)

# Include ZMK headers via the Zephyr build system
zephyr_include_directories($ENV{ZEPHYR_BASE}/../zmk/app/include)
zephyr_include_directories($ENV{ZEPHYR_BASE}/../zmk/app/drivers)



