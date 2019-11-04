#! /usr/bin/env bash

# NOTE: This script should only be run from the hebi_description/tools directory,
# otherwise paths break. 

# Call script to generate xacro/sdf from hrdf
#python3 urdf_generator.py ../hrdf/3-DoF_arm.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family HEBI --actuators Base Shoulder Elbow
python3 urdf_generator.py ../hrdf/a-2084-01.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1
python3 urdf_generator.py ../hrdf/a-2084-01-parallel-gripper.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1
python3 urdf_generator.py ../hrdf/a-2085-04.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1
python3 urdf_generator.py ../hrdf/a-2085-04-parallel-gripper.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1
python3 urdf_generator.py ../hrdf/a-2085-05.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1 wrist2
python3 urdf_generator.py ../hrdf/a-2085-05-parallel-gripper.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1 wrist2
python3 urdf_generator.py ../hrdf/a-2085-06.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1 wrist2 wrist3
python3 urdf_generator.py ../hrdf/a-2085-06-parallel-gripper.hrdf --urdfdir ../urdf/kits --sdfdir ../models --family Arm --actuators base shoulder elbow wrist1 wrist2 wrist3
