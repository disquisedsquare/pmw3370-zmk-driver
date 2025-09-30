zephyr_library()

# Source file
zephyr_library_sources(pmw3370.c)

# Include module's own headers
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Include ZMK headers via the Zephyr build system
zephyr_include_directories($ENV{ZEPHYR_BASE}/../zmk/app/include)
zephyr_include_directories($ENV{ZEPHYR_BASE}/../zmk/app/drivers)
zephyr_library_include_directories($ENV{ZEPHYR_BASE}/../zmk/app/include)
zephyr_library_include_directories($ENV{ZEPHYR_BASE}/../zmk/app/drivers)

