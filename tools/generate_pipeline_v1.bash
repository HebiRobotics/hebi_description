#! /usr/bin/env bash

# NOTE: This script should only be run from the hebi_description/tools directory,
# otherwise paths break. 

# first arg is hrdf file name, rest are actuator names
INPUT_FILE=$1
ACTUATORS="${@:2}"

# Call script to generate xacro/sdf from hrdf
python3 urdf_generator.py $INPUT_FILE --urdfdir ../urdf/kits --sdfdir ../models --family HEBI --actuators $ACTUATORS
# outfile contains the name of the outputted xacro
# needed because hrdf files are not named with the same convention as kits
# so the xacro needs to be named differently to match the moveit configs
# dict which holds this map is at the top of the python script

# cleanup intermediate file, since it's not really needed
# rm "$outfile.xacro.urdf"
