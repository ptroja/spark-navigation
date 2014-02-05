# Desc: Adaptation of Erratic file to a generic, circular robot
# Author: Joey Durham
# Date: August 2008

include "perfectlaser.inc"

define circlebot position
(
  # actual size
  size [0.5 0.5 0.1]

  # the center of rotation is offset from its center of area
  origin [0 0.0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 12.0 
	
  block(
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0 0.21]
  )

  # differential steering model
  drive "diff"

  # uncomment this line if you want to model real Erratics with SICK
  # lasers, where the laser is taller than the robot
  # laser_return 0
) 

define circlebot_perfect circlebot
(
	perfectlaser ( pose [0 0 0 0] )
)
