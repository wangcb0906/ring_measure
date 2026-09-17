cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(RingMeasurement)

if(MINGW)
# Force Eigen to use NDEBUG and disable debug checks to avoid runtime errors
add_definitions(-DEIGEN_NO_DEBUG -DNDEBUG)
endif()

set(CALI ring_cali)
set(EVALUATION ring_cali_eval)
set(MERGE merge)
set(CORE_LIB core_shared)
set(CORE core)
set(HELPER_LIB rm_helper)

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

set(RING_MEASURE_PCL_COMPONENTS
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
# Request ALL required PCL modules here
find_package(PCL 1.12 REQUIRED COMPONENTS ${RING_MEASURE_PCL_COMPONENTS})

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
file(GLOB CORE_SHARED "src/core/*.cpp")
if(MINGW)
    set(PCL_EXCLUDED_STATIC_LIBS)
    foreach(COMPONENT IN LISTS RING_MEASURE_PCL_COMPONENTS)
        list(APPEND PCL_EXCLUDED_STATIC_LIBS
            "-Wl,--exclude-libs,libpcl_${COMPONENT}.a"
        )
    endforeach()
    # Transitive PCL libraries used by the requested components
    list(APPEND PCL_EXCLUDED_STATIC_LIBS
        "-Wl,--exclude-libs,libpcl_octree.a"
        "-Wl,--exclude-libs,libpcl_io_ply.a"
    )
    list(REMOVE_DUPLICATES PCL_EXCLUDED_STATIC_LIBS)
    message(STATUS
        "PCL static libraries excluded from DLL exports: "
        "${PCL_EXCLUDED_STATIC_LIBS}"
    )

    add_library(${CORE} STATIC ${CORE_SHARED})
    target_link_options(${CORE} PRIVATE
        ${PCL_EXCLUDED_STATIC_LIBS}
    )

    add_library(${CORE_LIB} SHARED src/export/export.cpp)
    target_include_directories(${CORE_LIB} PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/export)
    target_link_libraries(${CORE_LIB} PRIVATE ${CORE})
    target_link_options(${CORE_LIB} PRIVATE
        ${PCL_EXCLUDED_STATIC_LIBS}
    )

    set(CORE_LINK ${CORE})
else()
    file(GLOB LINUX_SOURCES "src/linux/*.cpp")
    add_library(${CORE_LIB} SHARED ${CORE_SHARED} ${LINUX_SOURCES})

    set(CORE_LINK ${CORE_LIB})
endif()
target_include_directories(${CORE_LINK} PUBLIC 
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${PCL_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           )
target_link_libraries(${CORE_LINK} PUBLIC
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES}
                      spdlog::spdlog
                      ${JSONCPP_LIBRARY}
                      TIFF::tiff
                      )

# ------------------------------------------------------------
# Helper library
# ------------------------------------------------------------

file(GLOB HELPER_SHARED "src/aux/*.cpp")
add_library(${HELPER_LIB} SHARED ${HELPER_SHARED})
target_include_directories(${HELPER_LIB} PUBLIC 
                           ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ------------------------------------------------------------
# Applications
# ------------------------------------------------------------

set(APP_DIR "src/application")
add_executable (${CALI} ${APP_DIR}/ring_cali.cpp)
target_link_libraries(${CALI} PRIVATE ${CORE_LINK} ${HELPER_LIB})

add_executable(${EVALUATION} ${APP_DIR}/ring_cali_eval.cpp)
target_link_libraries(${EVALUATION} PRIVATE ${CORE_LINK} ${HELPER_LIB})

if(UNIX AND NOT APPLE)
    find_package(OpenMP REQUIRED)
    pkg_check_modules(ImageMagick REQUIRED Magick++)
    add_executable(${MERGE} ${APP_DIR}/merge.cpp)
    target_include_directories(${MERGE} PRIVATE ${ImageMagick_INCLUDE_DIRS})
    target_link_libraries(${MERGE} PRIVATE ${ImageMagick_LIBRARIES} OpenMP::OpenMP_CXX)
    target_compile_options(${MERGE} PRIVATE ${ImageMagick_CFLAGS_OTHER})
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
            "-DDYN=$<TARGET_FILE:${CORE_LIB}>"
            "-DAUX=$<TARGET_FILE:${HELPER_LIB}>"
            "-DDEPLOY_DIR=${DEPLOY_DIR}"
            "-DMINGW_BIN_DIR=${MINGW_BIN_DIR}"
            "-DMINGW=${MINGW}"
            -P "${CMAKE_SOURCE_DIR}/cmake/deploy.cmake"
        DEPENDS
            ${CALI}
            ${EVALUATION}
            ${CORE}
            ${CORE_LIB}
            ${HELPER_LIB}
        COMMENT "Collecting runtime DLL dependencies..."
    )
endif()

# To build the project

# for the calibration
# cmake -B build -DLOG_LEVEL=DEBUG -DCMAKE_BUILD_TYPE=Release
# cmake --build build -j --target ring_cali --clean-first
# To start a test
# cmake --build build -j --target ring_cali && ./ring_cali cali/20260622 -o auto -r 4_ND_20260622_4_19.ini
# cmake --build build -j --target ring_cali_eval  && ./ring_cali_eval cali/20260622/auto auto_cali_eval -b
# python visual/eval_bin.py ./cali/20260622/auto_cali_eval/
# ./ring_cali_eval cali/20260622/auto eval -e measure/20260326/r1 -b -t measure/20260326/config/11.ini
# python visual/eval_bin.py measure/20260326/r1/eval/ cali/20260622/auto