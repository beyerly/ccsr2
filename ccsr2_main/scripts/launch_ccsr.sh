#!/bin/bash


function waitForNode {
   while !(rosnode list | grep $1)
   do
      echo waiting
   done
}  

# roslaunch ccsr2_main rqt_consoles.launch &
roslaunch ccsr2_main ccsr2.launch &

waitForNode /camera/camera
roslaunch ccsr2_main image_annotation.launch &

waitForNode /rmotor_driver
waitForNode /lmotor_driver
rosservice call /enablermotor true
rosservice call /enablelmotor true

waitForNode /rplidarNode
rosservice call /stop_motor
