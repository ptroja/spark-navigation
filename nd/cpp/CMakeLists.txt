CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (nd_driver)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH "/opt/player/share/cmake/Modules")
INCLUDE (UsePlayerPlugin)

include_directories(../../common/clock)
PLAYER_ADD_PLUGIN_DRIVER (nd SOURCES geometria.cc nd.cc nd_plugin.cc ../../common/clock/clock.c)
