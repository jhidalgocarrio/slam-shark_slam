find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

message(STATUS "CMAKE SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE CURRENT SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
message(STATUS "GTSAM DIRECTORIES ${GTSAM_INCLUDE_DIR}")

rock_library(shark_slam
    HEADERS iShark.hpp
    SOURCES iShark.cpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG base-types gtsam)

