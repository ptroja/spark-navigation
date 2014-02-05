CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (vfh_driver)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH "/opt/player/share/cmake/Modules")
INCLUDE (UsePlayerPlugin)

include_directories(../../common/clock)
PLAYER_ADD_PLUGIN_DRIVER (vfh SOURCES vfh.cc vfh_algorithm.cc ../../common/clock/clock.c)
