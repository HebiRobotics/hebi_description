#! /usr/bin/env python3

import argparse
import sys
import os
from os.path import basename, splitext, join
import subprocess

from lxml import etree as ET

# this can be removed if hrdf filenames are changed to match moveit conventions
model_codes = {
    '3-DoF_arm': '3-DoF_arm',
    '4-DoF_arm_scara': 'A-2084-01',
    '4-DoF_arm': 'A-2085-04',
    '5-DoF_arm': 'A-2085-05',
    '5-DoF_arm_w_gripper': 'A-2085-05-parallel-gripper',
    '6-DoF_arm': 'A-2085-06',
    '6-DoF_arm_w_gripper': 'A-2085-06-parallel-gripper',
}

NS_XACRO = 'http://www.ros.org/wiki/xacro'

CONFIG_TEMPLATE = \
"""
<?xml version="1.0"?>
<model>
  <name>HEBI {0}</name>
  <version>1.0</version>
  <sdf version="1.5">{0}.sdf</sdf>

  <author>
    <name>Chris Bollinger</name>
    <email>chris@hebirobotics.com</email>
  </author>
</model>
"""


def get_names(num_names):
    names = []
    count = 0
    print(f"There are {num_names} actuators in the provided model. Please provide names for each.")
    for i in range(num_names):
        names.append(input('Input Actuator {} Name: '.format(i+1)))
    return names


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Convert HRDF files into xacro/sdf equivalent.')
    parser.add_argument('filename')
    parser.add_argument('--family', default='HEBI')
    parser.add_argument('--actuators', nargs="+", type=str, default=None)
    parser.add_argument('--urdfdir', default='.')
    parser.add_argument('--sdfdir', default='.')

    args = parser.parse_args()
    hrdf_file_name = args.filename

    model_name = splitext(basename(hrdf_file_name))[0]
    if model_name in model_codes:
        model_name = model_codes[model_name]

    ET.register_namespace('xacro', NS_XACRO)
    parser = ET.XMLParser(remove_blank_text=True)
    robot = ET.parse(hrdf_file_name, parser).getroot()

    num_actuators = len(list(robot.iter('actuator')))

    if args.family is None:
        family_name = input("Enter model's family name: ")
    else:
        family_name = args.family
    
    if args.actuators is None:
        actuator_names = get_names(num_actuators)
    elif len(args.actuators) == num_actuators:
        actuator_names = args.actuators
    else:
        print(args.actuators)
        print(type(args.actuators))
        msg = f'Given {len(args.actuators)} names, but there are {num_actuators} actuators in model'
        raise ValueError(msg)

    for idx, el in enumerate(robot.iter('actuator')):
        el.set('name', f'{family_name}/{actuator_names[idx]}')
        if 'type' in el.attrib:
            el.set('type', el.attrib['type'].replace('-', '_'))

    elmnts = list(robot)
    for idx, el in enumerate(elmnts):
        if el.tag == 'bracket':
            next_actuator_name = elmnts[idx+1].attrib['name'].split('/')[-1]
            el.set('name', f'{next_actuator_name}_bracket')
        elif el.tag == 'link':
            prev_actuator_name = elmnts[idx-1].attrib['name'].split('/')[-1]
            next_actuator_name = elmnts[idx+1].attrib['name'].split('/')[-1]
            el.set('name', f'{prev_actuator_name}_{next_actuator_name}')

        el.tag = '{'+NS_XACRO+'}' + el.tag

        # check that if twist/extension tags are present,
        # they are properly escaped in xacro
        props = ['twist', 'extension']
        for prop in props:
            if prop in el.attrib:
                val = el.attrib[prop]
                try:
                    float(val)
                except ValueError:
                    el.set(prop, f'${{{val}}}')


    # add a null_end_effector if chain ends in a non-gripper
    if elmnts[-1].tag != '{'+NS_XACRO+'}gripper':
        null_end = ET.Element('{'+NS_XACRO+'}null_end_effector', {'name': 'end_effector'})
        robot.append(null_end)

    # set child names for all elements
    elmnts = list(robot)
    for idx, el in enumerate(elmnts):
        if idx+1 == len(elmnts):
            break
        el.set('child', elmnts[idx+1].attrib['name'])

    robot.tag = ('robot')
    robot.set('name', model_name)

    world_joint = ET.Element('joint', {'name': 'world_joint', 'type': 'fixed'})
    ET.SubElement(world_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    ET.SubElement(world_joint, 'parent', {'link': 'world'})
    ET.SubElement(world_joint, 'child', {'link': f'{elmnts[0].attrib["name"]}/INPUT_INTERFACE'})

    robot.insert(0, world_joint)
    robot.insert(0, ET.Element('link', {'name': 'world'}))
    robot.insert(0, ET.Comment(f' HEBI {model_name} Style Arm Kit '))
    robot.insert(0, ET.Element('{'+NS_XACRO+'}include', {'filename': '$(find hebi_description)/urdf/hebi.xacro'}))

    ET.cleanup_namespaces(robot, top_nsmap={'xacro': NS_XACRO})
    xmlstr = ET.tostring(
        robot,
        pretty_print=True,
        xml_declaration=True,
        with_tail=False,
        encoding='UTF-8'
    )

    outfile = join(args.urdfdir, f'{model_name}.xacro')
    with open(outfile, 'wb') as f:
        f.write(xmlstr)

    # generate sdf and cleanup intermediate file
    with open(f'{model_name}.xacro.urdf', 'w') as urdf_file:
        xacro_cmd = ['xacro', '--xacro-ns', f'{outfile}']
        subprocess.call(xacro_cmd, stdout=urdf_file)

    # do some extra work here to match the file tree needed for gazebo
    model_folder = join(args.sdfdir, model_name)
    try:
        os.mkdir(model_folder , 0o755)
    except FileExistsError:
        # already exists? Don't care
        pass

    # create the model.config file
    with open(join(model_folder, "model.config"), 'w') as config_file:
        config_file.write(CONFIG_TEMPLATE.format(model_name))

    with open(join(model_folder, f'{model_name}.sdf'), 'w') as sdf_file:
        sdf_cmd = ['gz', 'sdf', '-p', f'{model_name}.xacro.urdf']
        subprocess.call(sdf_cmd, stdout=sdf_file)

    cleanup_cmd = ['rm', f'{model_name}.xacro.urdf']
    subprocess.call(cleanup_cmd)

