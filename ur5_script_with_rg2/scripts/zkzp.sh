#!/bin/bash
# by wishchin 2020-05-12  version 1.0
## # Option “-x” is deprecated and might be removed in a later version of gnome-terminal.
## Use “-- ” to terminate the options and put the command line to execute after it.
 
#start
source ~/catkin_ws/devel/setup.bash


{
#cloud process!
gnome-terminal -t "new_ibvs_three_features" -x bash -c "rosrun ur5_vs new_ibvs_three_features;exec bash"
}&

sleep 22s

{
#python service......
gnome-terminal -t "python_trans" -x bash -c "python test_main.py;exec bash"
}&



#end 
