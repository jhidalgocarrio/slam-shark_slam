#message(STATUS "CMAKE SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
#message(STATUS "CMAKE CURRENT SOURCE DIRECTORY ${CMAKE_SOURCE_DIR}")
#message(STATUS "GTSAM DIRECTORIES ${GTSAM_INCLUDE_DIR}")

find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
find_package(Boost COMPONENTS filesystem system REQUIRED)
add_definitions(${GTSAM_CFLAGS_OTHER})
include_directories(${GTSAM_INCLUDE_DIR})
message(STATUS "GTSAM_INCLUDE_DIR ${GTSAM_INCLUDE_DIR}")
link_directories(${GTSAM_LIBRARY_DIRS})
message(STATUS "GTSAM_LIBRARY_DIRS ${GTSAM_LIBRARY_DIRS}")

rock_library(shark_slam
    SOURCES iShark.cpp
    HEADERS iShark.hpp
    DEPS_PKGCONFIG base-lib base-types gtsam)
