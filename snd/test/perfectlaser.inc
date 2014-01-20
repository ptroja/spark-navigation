# Desc: Device definitions for a 360 fov laser
# Author: Joey Durham
# Date: August 2008

define perfectsensor sensor
(
  range [ 0.0 5.2 ]
  #range_max 20.0

  fov 180.0
  #samples 683
  samples 1000
)

define perfectcommon ranger
(
  color "yellow"
  size [ 0.04 0.04 0.1 ]
)

define perfectlaser perfectcommon ( perfectsensor() )
