## ./targets/pic_32bit/mikroe/pic32/CMakeLists.txt
#  ALL MODULES
add_subdirectory(src/hal_ll)
#  SEPERATE MODULES
add_subdirectory(src/adc)
add_subdirectory(src/gpio)
add_subdirectory(src/i2c)
add_subdirectory(src/spi_master)
add_subdirectory(src/tim)
add_subdirectory(src/uart)
#  PERIPHERAL PIN SELECT
add_subdirectory(src/pps)
add_subdirectory(src/one_wire)
add_subdirectory(src/rtc)
