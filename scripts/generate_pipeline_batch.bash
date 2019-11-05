#! /usr/bin/env bash

PKG_DIR=$(rospack find hebi_description)
if [ -z "$PKG_DIR" ]
then
  SDFFLAG="--nosdf"
  PKG_DIR="."
fi
# Call script to generate xacro/sdf from hrdf
#python3 urdf_generator.py ../hrdf/3-DoF_arm.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family HEBI --actuators Base Shoulder Elbow
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2084-01.hrdf"                  $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2085-04.hrdf"                  $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2085-05.hrdf"                  $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1 wrist2
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2085-06.hrdf"                  $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1 wrist2 wrist3
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2084-01-parallel-gripper.hrdf" $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2085-04-parallel-gripper.hrdf" $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2085-05-parallel-gripper.hrdf" $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1 wrist2
python3 "$PKG_DIR/scripts/urdf_generator.py" "$PKG_DIR/hrdf/a-2085-06-parallel-gripper.hrdf" $SDFFLAG --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators base shoulder elbow wrist1 wrist2 wrist3
