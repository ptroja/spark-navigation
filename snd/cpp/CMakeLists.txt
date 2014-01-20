CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (snd_driver)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH "/opt/player/share/cmake/Modules")
INCLUDE (UsePlayerPlugin)

PLAYER_ADD_PLUGIN_DRIVER (snd SOURCES snd_driver.cc snd_algo.cc spaces.cc)
