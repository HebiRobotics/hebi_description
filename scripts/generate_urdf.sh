#!/bin/bash

# NOTE: If the ROS environment is not sourced, this script should be run from hebi_description/scripts,
# otherwise paths break.

PKG_DIR=$1
if [ -z "$PKG_DIR" ]; then
    echo "Please provide the package directory for "hebi_description" as an argument"
    exit 1
fi

CFG_DIR=$PKG_DIR/config

# Loop through all .cfg.yaml files in the config directory
for cfg_file in "$CFG_DIR"/*.cfg.yaml; do
    # Check if file exists (this prevents errors if no matching files are found)
    if [ -f "$cfg_file" ]; then
        # Extract just the filename without path
        filename=$(basename "$cfg_file")
        # Run the command
        python3 "$PKG_DIR/scripts/urdf_generator.py" "$cfg_file" --outputdir "$PKG_DIR/urdf/kits"
    fi
done

echo "All files processed."