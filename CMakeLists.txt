cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(RingMeasurement)

set(CALI ring_cali)
set(MEASURE ring_cali_eval)
set(CORE_LIB core_shared)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")


find_package(spdlog)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JsonCpp REQUIRED IMPORTED_TARGET jsoncpp)

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

# SPDLOG_LEVEL_TRACE
# SPDLOG_LEVEL_DEBUG
# SPDLOG_LEVEL_INFO
# SPDLOG_LEVEL_WARN
# SPDLOG_LEVEL_ERROR
# SPDLOG_LEVEL_CRITICAL
add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)

# Add RING_MEASURE_VERIFY_ONLY to enbale the verify mode.
# add_compile_definitions(RING_MEASURE_CALI_ONLY)
# add_compile_definitions(RING_MEASURE_VERIFY_ONLY)

# Put the output binaries in the root directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")

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
                      PkgConfig::JsonCpp
                       )
                       
set(APP_DIR "src/application")
add_executable (${CALI} ${APP_DIR}/ring_cali.cpp)
target_link_libraries(${CALI} PRIVATE ${CORE_LIB})

add_executable(${MEASURE} ${APP_DIR}/ring_cali_eval.cpp)
target_link_libraries(${MEASURE} PRIVATE ${CORE_LIB})

# To build the project

# for the calibration
# cmake -B build
# cmake --build build --target ring_cali -j --clean-first
# To start a test
# cmake --build build --target ring_cali -j && ./ring_cali test/20260622 -r 4_ND_20260622_4_19.ini
# To check the quality of a construction, note that the scans are not well aligned.
# magick composite -blend 50% result/GR_4_ND_20260622_4_19/in_test_denoise/line_13_in_recon.png result/GR_4_ND_20260622_4_19/out_test_denoise/line_11_out_recon.png  output.png