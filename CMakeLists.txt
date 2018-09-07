# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
set(ROCK_USE_CXX11 TRUE)
set(ROCK_TEST_ENABLED ON CACHE BOOL "Build with tests")
rock_init(shark_slam 0.1)
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
rock_standard_layout()


