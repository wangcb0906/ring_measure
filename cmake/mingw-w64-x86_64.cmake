# Cross-compilation: Arch Linux -> Windows x86-64
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# MinGW-w64 compiler
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER x86_64-w64-mingw32-windres)

# Tell CMake where to search for target libraries/packages.
set(CMAKE_FIND_ROOT_PATH
    /usr/x86_64-w64-mingw32
)

# Programs such as cmake, ninja, python, etc. run on Arch Linux.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Headers, libraries and packages must come from the Windows target.
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force SSE2 alignment and disable AVX to prevent Windows stack crashes
# On Windows, the stack is 16Byte aligned, but AVX requires 32Byte alignment. 
# If the stack is not aligned, it will crash with a segmentation fault.
# AVX works only on Linux.
set(CMAKE_C_FLAGS_INIT "-mno-avx -msse2 -mstackrealign")
set(CMAKE_CXX_FLAGS_INIT "-mno-avx -msse2 -mstackrealign")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mno-avx -msse2 -mstackrealign" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-avx -msse2 -mstackrealign" CACHE STRING "" FORCE)

set(OpenCV_DIR
    "/usr/x86_64-w64-mingw32/static/lib/cmake/opencv5"
    CACHE PATH "OpenCV CMake package directory"
)

# To build for Windows, run the following commands in the project root directory:
# find build-win/ -mindepth 1 -maxdepth 1 ! -name 'deploy' -exec rm -rf {} +
# cmake -S . -B build-win -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/mingw-w64-x86_64.cmake" -DLOG_LEVEL=DEBUG -DCMAKE_BUILD_TYPE=Debug
# cmake --build build-win --target deploy -j