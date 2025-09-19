cmake_minimum_required(VERSION 3.13.1)
set(PROJ_NAME pmw3370_driver)
project(${PROJ_NAME})

# Zephyr build helper
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pmw3370_driver)

target_sources(app PRIVATE src/pmw3370.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

