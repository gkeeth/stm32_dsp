Steps to build:

1. Configure CMake.

In this directory, run:
cmake -DROOT="<absolute path to CMSIS_5>" -DCMAKE_PREFIX_PATH="/usr/" -DCMAKE_TOOLCHAIN_FILE="<absolute path to CMSIS_5>/CMSIS/DSP/gcc.cmake" -DARM_CPU="cortex-m4" -G "Unix Makefiles" ..

(from: https://github.com/ARM-software/CMSIS_5/blob/develop/CMSIS/DSP/README.md)

2. Build.

In this directory, run:
make VERBOSE=1
