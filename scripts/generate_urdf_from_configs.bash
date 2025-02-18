#! /usr/bin/env bash

# NOTE: If the ROS environment is not sourced, this script should be run from hebi_description/scripts,
# otherwise paths break.

PKG_DIR=$1
if [ -z "$PKG_DIR" ]; then
    echo "Please provide the package directory for "hebi_description" as an argument"
    exit 1
fi
if [[ "$PKG_DIR" == */ ]]; then
    PKG_DIR=${PKG_DIR%/}
fi

CONFIG_LOCATION=$PKG_DIR/config/arms

echo $PKG_DIR
echo $CONFIG_LOCATION

# For all files in the config directory in format "<alphabet>-<digit*4>-<digit*2>.cfg.yaml", call the script to generate URDF
for file in "$CONFIG_LOCATION"/*-*-*.cfg.yaml; do
    # Generate the URDF for this file
    python3 "$PKG_DIR/scripts/urdf_generator.py" "$file" --outputdir "$PKG_DIR/urdf/kits"
done

# Check for .hrdf files in CONFIG_LOCATION/hrdf and generate URDFs if corresponding .cfg.yaml files do not exist
HRDF_LOCATION=$CONFIG_LOCATION/hrdf
for hrdf_file in "$HRDF_LOCATION"/*.hrdf; do
    base_name=$(basename "$hrdf_file" .hrdf)
    cfg_file="$CONFIG_LOCATION/$base_name.cfg.yaml"
    if [ ! -f "$cfg_file" ]; then
        # Generate the URDF for this .hrdf file
        python3 "$PKG_DIR/scripts/urdf_generator.py" "$hrdf_file" --outputdir "$PKG_DIR/urdf/kits"
    fi
done
