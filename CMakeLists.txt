cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(RingMeasurement)

set(CALI ring_cali)
set(EVALUATION ring_cali_eval)
set(MERGE merge)
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
find_package(OpenCV 5 REQUIRED COMPONENTS core imgproc)
find_package(TIFF REQUIRED)
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
add_compile_definitions(SPDLOG_FMT_EXTERNAL)
if(NOT RING_MEASURE_SAVE_PLY_TYPE)
  set(RING_MEASURE_SAVE_PLY_TYPE RING_MEASURE_SAVE_BIN CACHE STRING "" FORCE)
endif()
add_compile_definitions(RING_MEASURE_SAVE_PLY_TYPE=${RING_MEASURE_SAVE_PLY_TYPE})
if(MINGW)
add_compile_definitions(
    PCL_NO_PRECOMPILE
)
endif()

# ------------------------------------------------------------
# Core library
# ------------------------------------------------------------

file(GLOB CORE_SHARED "src/*.cpp")
if(UNIX AND NOT APPLE)
    file(GLOB LINUX_SHARED "src/linux/*.cpp")
    list(APPEND CORE_SHARED ${LINUX_SHARED})
endif()
add_library(${CORE_LIB} SHARED ${CORE_SHARED})

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
                      TIFF::tiff
                       )

# ------------------------------------------------------------
# Applications
# ------------------------------------------------------------

set(APP_DIR "src/application")
add_executable (${CALI} ${APP_DIR}/ring_cali.cpp)
target_link_libraries(${CALI} PRIVATE ${CORE_LIB})

add_executable(${EVALUATION} ${APP_DIR}/ring_cali_eval.cpp)
target_link_libraries(${EVALUATION} PRIVATE ${CORE_LIB})

if(UNIX AND NOT APPLE)
    find_package(OpenMP REQUIRED)
    pkg_check_modules(ImageMagick REQUIRED Magick++)
    add_executable(${MERGE} ${APP_DIR}/merge.cpp)
    target_include_directories(${MERGE} PRIVATE ${ImageMagick_INCLUDE_DIRS})
    target_link_libraries(${MERGE} PRIVATE ${ImageMagick_LIBRARIES} OpenMP::OpenMP_CXX)
    target_compile_options(merge PRIVATE ${ImageMagick_CFLAGS_OTHER})
endif()

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
            "-DAPP2=$<TARGET_FILE:${EVALUATION}>"
            "-DDEPLOY_DIR=${DEPLOY_DIR}"
            "-DMINGW_BIN_DIR=${MINGW_BIN_DIR}"
            "-DMINGW=${MINGW}"
            -P "${CMAKE_SOURCE_DIR}/cmake/deploy.cmake"
        DEPENDS
            ${CALI}
            ${EVALUATION}
        COMMENT "Collecting runtime DLL dependencies..."
    )
endif()

# To build the project

# for the calibration
# cmake -B build -DLOG_LEVEL=DEBUG -DCMAKE_BUILD_TYPE=Release
# cmake --build build -j --target ring_cali --clean-first
# To start a test
# cmake --build build -j --target ring_cali && ./ring_cali cali/20260622 -o auto -r 4_ND_20260622_4_19.ini
# cmake --build build -j --target ring_cali_eval  && ./ring_cali_eval cali/20260622/auto auto_cali_eval -bm
# python visual/eval_bin.py ./cali/20260622/auto_cali_eval/
# ./ring_cali_eval cali/20260622/auto eval -e measure/20260326/r1 -b -t measure/20260326/config/11.ini
# python visual/eval_bin.py measure/20260326/r1/eval/ cali/20260622/auto