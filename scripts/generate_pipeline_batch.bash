#! /usr/bin/env bash

# NOTE: If the ROS environment is not sourced, this script should be run from hebi_description/scripts,
# otherwise paths break.

PKG_DIR=$1
if [ -z "$PKG_DIR" ]; then
    echo "Please provide the package directory for "hebi_description" as an argument"
    exit 1
fi

HRDF_LOCATION=$PKG_DIR/config

echo $PKG_DIR
echo $HRDF_LOCATION

# Call script to generate xacro/sdf from hrdf
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2084-01.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-04.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-05.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-06.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2 J6_wrist3
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2084-01-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-04-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-05-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-06-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2 J6_wrist3
