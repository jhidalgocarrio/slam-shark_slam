#message(STATUS "CMAKE SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
#message(STATUS "CMAKE CURRENT SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
#message(STATUS "GTSAM DIRECTORIES ${GTSAM_INCLUDE_DIR}")

find_package(GTSAMCMakeTools)
find_package(Boost 1.43 COMPONENTS filesystem system REQUIRED)

rock_library(shark_slam
    SOURCES iShark.cpp
    HEADERS iShark.hpp
    DEPS_PKGCONFIG base-lib gtsam)
