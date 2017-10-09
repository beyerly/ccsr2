#!/bin/bash


# roslaunch ccsr2_main rqt_consoles.launch
# roslaunch ccsr2_main ccsr2.launch


rosservice call /enablelmotor truesuccess: False
rosservice call /enablermotor true
rosservice call /stop_motor
