#! /usr/bin/env bash

# NOTE: This script should only be run from the hebi_description/tools directory,
# otherwise paths break. 

# Call script to generate xacro/sdf from hrdf
python3 urdf_generator.py ../hrdf/4-DoF_arm_scara.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family HEBI --actuators Base Shoulder Elbow Wrist1
python3 urdf_generator.py ../hrdf/4-DoF_arm.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family HEBI --actuators Base Shoulder Elbow Wrist1
python3 urdf_generator.py ../hrdf/5-DoF_arm.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family HEBI --actuators Base Shoulder Elbow Wrist1 Wrist2
python3 urdf_generator.py ../hrdf/6-DoF_arm.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family HEBI --actuators Base Shoulder Elbow Wrist1 Wrist2 Wrist3
