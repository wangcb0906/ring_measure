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

# To build for Windows, run the following commands in the project root directory:
# rm -rf build-win
# cmake -S . -B build-win -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/mingw-w64-x86_64.cmake"
# cmake --build build-win --target deploy -j
