cmake_minimum_required (VERSION 3.14)

project (testcmsisdsp VERSION 0.1)

set(DSP ${ROOT}/CMSIS/DSP)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast")

list(APPEND CMAKE_MODULE_PATH ${DSP})

add_subdirectory(${DSP}/Source bin_dsp)

