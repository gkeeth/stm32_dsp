cmake_minimum_required (VERSION 3.14)

project (testcmsisdsp VERSION 0.1)

set(DSP ${ROOT}/CMSIS/DSP)

list(APPEND CMAKE_MODULE_PATH ${DSP})

add_subdirectory(${DSP}/Source bin_dsp)

