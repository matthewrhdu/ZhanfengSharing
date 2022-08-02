#!/usr/bin/env zsh

if [ $# == 0 ]
then
    k=2
else
    k=$1
fi

if [ $k -ne 1 ]
then
    colcon build --packages-select data_structures
fi

if [ $k -ne 0 ]
then
    colcon build --packages-select sample_server
fi
