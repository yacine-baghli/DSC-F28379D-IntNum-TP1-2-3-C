cmake_minimum_required(VERSION 3.10)
project(main_epwm_adc)

set(CMAKE_CXX_STANDARD 11)

include_directories(c:/ti/c2000/C2000Ware_5_02_00_00/device_support/f2837xd/common/include)
include_directories(c:/ti/c2000/C2000Ware_5_02_00_00/device_support/f2837xd/headers/include)

add_executable(main_epwm_adc main_epwm_adc.c c:/ti/c2000/C2000Ware_5_02_00_00/device_support/f2837xd/headers/source/F2837xD_GlobalVariableDefs.c)

