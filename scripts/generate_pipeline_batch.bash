#! /usr/bin/env bash

# NOTE: If the ROS environment is not sourced, this script should be run from hebi_description/scripts,
# otherwise paths break.

PKG_DIR=$1
if [ -z "$PKG_DIR" ]; then
    echo "Please provide the package directory for "hebi_description" as an argument"
    exit 1
fi

CONFIG_LOCATION=$PKG_DIR/config/arms

echo $PKG_DIR
echo $CONFIG_LOCATION

# For all files in the config directory in format "<alphabet>-<digit*4>-<digit*2>.cfg.yaml", call the script to generate URDF
for file in "$CONFIG_LOCATION"/*-*-*.cfg.yaml; do
    # Generate the URDF for this file
    python3 "$PKG_DIR/scripts/urdf_generator.py" "$file" --outputdir "$PKG_DIR/urdf/kits"
done
