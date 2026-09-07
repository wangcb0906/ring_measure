cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(RingMeasurement)

set(CALI ring_cali)
set(MEASURE ring_cali_eval)
set(CORE_LIB core_shared)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(MINGW)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()

# ------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------

find_package(spdlog REQUIRED)
find_package(OpenCV 5 REQUIRED COMPONENTS core imgproc imgcodecs)
if(MINGW)
    set(Boost_USE_STATIC_LIBS OFF)
    set(Boost_USE_STATIC_RUNTIME OFF)
    find_file(JSONCPP_LIBRARY
        NAMES libjsoncpp.dll.a
        PATHS /usr/x86_64-w64-mingw32/lib
        NO_DEFAULT_PATH
        REQUIRED
    )
    message(STATUS "Using JsonCpp library: ${JSONCPP_LIBRARY}")
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JsonCpp REQUIRED IMPORTED_TARGET jsoncpp)
    set(JSONCPP_LIBRARY PkgConfig::JsonCpp)
endif()

# Request ALL required PCL modules here
find_package(PCL 1.12 REQUIRED COMPONENTS 
    common 
    io 
    filters 
    segmentation 
    search 
    features 
    kdtree 
    sample_consensus
    surface
    ml
)

# (TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF)
set(LOG_LEVEL INFO CACHE STRING "")
add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_${LOG_LEVEL})

add_compile_definitions(
    SPDLOG_FMT_EXTERNAL
    # PCL_NO_PRECOMPILE
)

# ------------------------------------------------------------
# Core library
# ------------------------------------------------------------

file(GLOB CORE_SHARED "src/*.cpp")
add_library(${CORE_LIB} OBJECT ${CORE_SHARED})

target_include_directories(${CORE_LIB} PUBLIC 
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${PCL_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           )

target_link_libraries(${CORE_LIB} PUBLIC
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES}
                      spdlog::spdlog
                      ${JSONCPP_LIBRARY}
                       )

# ------------------------------------------------------------
# Applications
# ------------------------------------------------------------

set(APP_DIR "src/application")
add_executable (${CALI} ${APP_DIR}/ring_cali.cpp)
target_link_libraries(${CALI} PRIVATE ${CORE_LIB})

add_executable(${MEASURE} ${APP_DIR}/ring_cali_eval.cpp)
target_link_libraries(${MEASURE} PRIVATE ${CORE_LIB})


# ------------------------------------------------------------
# Deployment on Windows
# ------------------------------------------------------------
if(MINGW)
    set(DEPLOY_DIR "${CMAKE_BINARY_DIR}/deploy")
    list(GET CMAKE_FIND_ROOT_PATH 0 MINGW_ROOT)
    set(MINGW_BIN_DIR "${MINGW_ROOT}/bin")
    add_custom_target(deploy
        COMMAND ${CMAKE_COMMAND}
            "-DAPP1=$<TARGET_FILE:${CALI}>"
            "-DAPP2=$<TARGET_FILE:${MEASURE}>"
            "-DDEPLOY_DIR=${DEPLOY_DIR}"
            "-DMINGW_BIN_DIR=${MINGW_BIN_DIR}"
            "-DMINGW=${MINGW}"
            -P "${CMAKE_SOURCE_DIR}/cmake/deploy.cmake"
        DEPENDS
            ${CALI}
            ${MEASURE}
        COMMENT "Collecting runtime DLL dependencies..."
    )
endif()

# Add RING_MEASURE_VERIFY_ONLY to enbale the verify mode.
add_compile_definitions(RING_MEASURE_CALI_ONLY)
# add_compile_definitions(RING_MEASURE_VERIFY_ONLY)

# To build the project

# for the calibration
# cmake -B build -DLOG_LEVEL=DEBUG -DCMAKE_BUILD_TYPE=Release
# cmake --build build --target ring_cali -j --clean-first
# To start a test
# cmake --build build --target ring_cali -j && ./ring_cali cali/20260622 -r 4_ND_20260622_4_19.ini
# cmake --build build --target ring_cali_eval -j && ./ring_cali_eval cali/20260622
# python visual/eval_bin.py ./cali/20260622/result_cali_eval/
# To check the quality of a construction, note that the scans are not well aligned.
# magick composite -blend 50% result/GR_4_ND_20260622_4_19/in_test_denoise/line_13_in_recon.png \
#     result/GR_4_ND_20260622_4_19/out_test_denoise/line_11_out_recon.png  output.png