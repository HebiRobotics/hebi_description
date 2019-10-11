#! /usr/bin/env bash

ACTUATORS="${@:2}"

outfile=$(python3 urdf_generator.py $1 --family HEBI --actuators $ACTUATORS)
xacro --xacro-ns "$outfile.xacro" > "$outfile.xacro.urdf"
gz sdf -p "$outfile.xacro.urdf" > "$outfile.sdf"
