cmake_minimum_required(VERSION 3.20)
project(gz-mooring VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(gz-cmake REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz-common REQUIRED COMPONENTS profiler)
find_package(gz-msgs REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-sim REQUIRED)

set(MOORDYN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../MoorDyn")
find_library(MOORDYN_LIBRARY moordyn
  PATHS
    "${MOORDYN_DIR}/build/source"
    "${MOORDYN_DIR}/build"
    "${MOORDYN_DIR}/build/lib"
  NO_DEFAULT_PATH
)
find_path(MOORDYN_INCLUDE_DIR MoorDyn2.h
  PATHS "${MOORDYN_DIR}/source"
  NO_DEFAULT_PATH
)

if(NOT MOORDYN_LIBRARY)
  message(FATAL_ERROR
    "libmoordyn non trovata. Buildala con:\n"
    "  cmake -S src/MoorDyn -B src/MoorDyn/build "
    "-DPYTHON_WRAPPER=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release\n"
    "  make -C src/MoorDyn/build -j4")
endif()
if(NOT MOORDYN_INCLUDE_DIR)
  message(FATAL_ERROR "Header MoorDyn2.h non trovati in ${MOORDYN_DIR}/source")
endif()

message(STATUS "MoorDyn library : ${MOORDYN_LIBRARY}")
message(STATUS "MoorDyn headers : ${MOORDYN_INCLUDE_DIR}")

add_library(MooringSystem SHARED src/Mooring.cc)

target_include_directories(MooringSystem
  PRIVATE
    ${MOORDYN_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(MooringSystem
  PRIVATE
    gz-sim::gz-sim
    gz-transport::gz-transport
    gz-msgs::gz-msgs
    gz-math::gz-math
    gz-plugin::gz-plugin-register
    gz-common::gz-common
    ${MOORDYN_LIBRARY}
)

install(TARGETS MooringSystem LIBRARY DESTINATION lib)
