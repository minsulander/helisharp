#!/bin/bash -e
dir=$(dirname $0)
mdtool build -p:HeliSharpTool
HeliSharpTool/bin/Debug/HeliSharpTool.exe sim --control latcyclic --deflection -2.5 >$dir/../validation/a109/data/simdata_lat.csv
HeliSharpTool/bin/Debug/HeliSharpTool.exe sim --control longcyclic --deflection 2.2 >$dir/../validation/a109/data/simdata_long.csv

cd $dir/octave
octave -q simdata_lat_compare.m
octave -q simdata_long_compare.m
