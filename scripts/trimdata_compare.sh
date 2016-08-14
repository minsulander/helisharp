#!/bin/bash -e
dir=$(dirname $0)
mdtool build -p:HeliSharpTool
HeliSharpTool/bin/Debug/HeliSharpTool.exe trim >$dir/../validation/a109/data/trimdata.csv
cd $dir/octave
octave -q trimdata_compare.m
