#! /usr/bin/env bash

# NOTE: This script should only be run from the hebi_description/tools directory,
# otherwise paths break. 

# Call script to generate xacro/sdf from hrdf
python3 urdf_generator.py ../hrdf/A-2084-01.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1
python3 urdf_generator.py ../hrdf/A-2085-04.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1
python3 urdf_generator.py ../hrdf/A-2085-05.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2
python3 urdf_generator.py ../hrdf/A-2085-06.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2 J6_wrist3
