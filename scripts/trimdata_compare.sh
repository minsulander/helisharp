#!/bin/bash -e
cd $(dirname $0)/../HeliSharpTool
dotnet run --no-build -- trim >../validation/a109/data/trimdata.csv
cd ../scripts/octave
octave -q trimdata_compare.m
