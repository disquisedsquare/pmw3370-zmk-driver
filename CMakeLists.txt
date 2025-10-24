zephyr_include_directories(.)
zephyr_include_directories(dts/bindings)

if(CONFIG_PMW3370)
  zephyr_library()
  zephyr_library_sources(src/pmw3370.c)
endif()
