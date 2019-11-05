#! /usr/bin/env bash

PACKAGE_DIR=$(rospack find hebi_description)
# first arg is hrdf file name, rest are actuator names
INPUT_FILE=$1
ACTUATORS="${@:2}"

# Call script to generate xacro/sdf from hrdf
python3 "$PACKAGE_DIR/scripts/urdf_generator.py" $INPUT_FILE --urdfdir "$PACKAGE_DIR/urdf/kits" --sdfdir "$PACKAGE_DIR/models" --family HEBI --actuators $ACTUATORS
