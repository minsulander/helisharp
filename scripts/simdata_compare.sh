#!/bin/bash -e
cd $(dirname $0)/../HeliSharpTool
dotnet run --no-build -- sim --control latcyclic --deflection -2.5 >../validation/a109/data/simdata_lat.csv
dotnet run --no-build -- sim --control longcyclic --deflection 2.2 >../validation/a109/data/simdata_long.csv

cd ../scripts/octave
octave -q simdata_lat_compare.m
octave -q simdata_long_compare.m
