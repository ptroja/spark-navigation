# Desc: Device definitions for Erratic robots.
# Author: Kurt Konolige (edited by Joey Durham)
# Date: October 2006

include "urglaser.inc"

# Erratic ERA platform, Rev E
define erratic position
(
  # actual size
  size [0.44 0.44 0.1]

  # the center of rotation is offset from its center of area
  origin [-0.08 0.0 0 0]

  # draw a nose on the robot so we can see which way it points
  #gui_nose 1

  # estimated mass in KG
  mass 12.0 
	
  # this polygon approximates the shape of erratic
  block(
    points 8
    point[0] [  0.19  0.11 ]
    point[1] [  0.13  0.19 ]
    point[2] [ -0.13  0.19 ]
    point[3] [ -0.19  0.10 ]
    point[4] [ -0.19 -0.10 ]
    point[5] [ -0.13 -0.19 ]
    point[6] [  0.13 -0.19 ]
    point[7] [  0.19 -0.11 ]
    z [0 0.21]
  )

  # differential steering model
  drive "diff"

  # uncomment this line if you want to model real Erratics with SICK
  # lasers, where the laser is taller than the robot
  #laser_return 0

) 

#define erratic_battery erratic
#(
#  power( pose [-0.14 0 0] capacity 1000 probe_range 0.50  give 0 give_rate 100 take_rate 100 )
#)

define erratic_urg erratic
(
	urg_laser( pose [0 0 0 0] )
)
