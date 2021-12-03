cmake_minimum_required(VERSION 3.10)

project(GraphAlgorithm CXX)

set(GRAPH_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Pre-required lib
find_package(Boost)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  Message("Boost Path: " ${Boost_INCLUDE_DIRS})
endif()

# includes
include_directories(AFTER "${GRAPH_DIR}/include")

# compile flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pipe -Wall -Werror -fPIC -fstack-protector-all")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pipe -Wall -Werror -fPIC -fstack-protector-all")

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  message("-- Build in Debug mode")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -rdynamic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -rdynamic")
  add_definitions(-DENABLE_DEBUG)
else()
  message("-- Build in Release mode")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
endif()


file(GLOB_RECURSE COMPILE_SRCS ${GRAPH_DIR}/*.cc)

add_library(GraphAlgorithm SHARED ${COMPILE_SRCS})

# Tests
set(TEST_EXES "")
file(GLOB TEST_SRCS ${GRAPH_DIR}/tests/*.cc)
foreach(__srcpath ${TEST_SRCS})
  get_filename_component(__srcname ${__srcpath} NAME)
  string(REPLACE ".cc" "" __execname ${__srcname})
  add_executable(${__execname} ${__srcpath})
  message("add exe test" ${__execname})
  target_link_libraries(${__execname} PRIVATE GraphAlgorithm pthread dl)
endforeach()
