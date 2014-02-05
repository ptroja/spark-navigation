define urg_sensor sensor
(
  range [0.0 5.2]
  
  fov 240.0
  #fov 360.0
  samples 683
  #samples 1024
) 

define urg_common ranger
(
  color "yellow"
  size [ 0.04 0.04 0.06 ]
)

define urg_laser urg_common ( urg_sensor() )
