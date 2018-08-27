rock_library(shark_slam
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(shark_slam_bin Main.cpp
    DEPS shark_slam)

