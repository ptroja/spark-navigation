#PLAYERDRIVER_REQUIRE_PKG (wavefront build_wavefront gdk-pixbuf-2.0

CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (wavefront_driver)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH "/opt/player/share/cmake/Modules")
INCLUDE (UsePlayerPlugin)

#include_directories(../../common/clock)
PLAYER_ADD_PLUGIN_DRIVER (wavefront SOURCES plan.cc plan_plan.cc plan_waypoint.cc wavefront.cc plan_control.cc)
