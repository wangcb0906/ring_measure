#!/bin/bash

# clean the build-win directory except for the deploy folder
find build-win/ -mindepth 1 -maxdepth 1 ! -name 'deploy' -exec rm -rf {} +

# Declare the original list of log levels
log_levels=("OFF" "CRITICAL" "ERROR" "WARN" "INFO" "DEBUG" "TRACE")

# Loop backwards through the array using indices
for ((i=0; i<${#log_levels[@]}; i++)); do
    level="${log_levels[i]}"
    cmake -S . -B build-win -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/mingw-w64-x86_64.cmake" -DLOG_LEVEL=${level} -DCMAKE_BUILD_TYPE=Release
    cmake --build build-win --target deploy -j
    echo "Compiled with Log Level: Index: $i | Log Level: $level"

    target_dir="build-win/deploy/${i}.${level}"
    echo "Target directory is: '$target_dir'"
    mkdir -p ${target_dir}

    # Check if the file exists before moving, otherwise search for it
    if [ -f "build-win/deploy/libcore_shared.dll" ]; then
        mv "build-win/deploy/libcore_shared.dll" "${target_dir}/"
        echo "Moved to ${target_dir}/"
    else
        echo "Warning: libcore_shared.dll not found in build-win/deploy/. Actual locations:"
        find build-win -name "libcore_shared.dll"
    fi

    sleep 5
done