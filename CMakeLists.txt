cmake_minimum_required(VERSION 3.13.4)

PROJECT(eigen_pg)
IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  ADD_DEFINITIONS(-std=c++17)
ENDIF()

include_directories(include)
file(GLOB SOURCES "src/*.cpp")
ADD_EXECUTABLE(test ${SOURCES})
