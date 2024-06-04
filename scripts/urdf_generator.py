#! /usr/bin/env python3

import argparse
import sys
import os
from os.path import basename, splitext, join
import subprocess
import re

from lxml import etree as ET

NS_XACRO = 'http://wiki.ros.org/xacro'

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
    print("There are {} actuators in the provided model. Please provide names for each.".format(num_names))
    for i in range(num_names):
        names.append(input('Input Actuator {} Name: '.format(i+1)))
    return names


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Convert HRDF files into xacro/sdf equivalent.')
    parser.add_argument('filename')
    parser.add_argument('--actuators', nargs="+", type=str, default=None)
    parser.add_argument('--urdfdir', default='.')

    args = parser.parse_args()
    hrdf_file_name = args.filename

    model_name = splitext(basename(hrdf_file_name))[0]

    ET.register_namespace('xacro', NS_XACRO)
    parser = ET.XMLParser(remove_blank_text=True, remove_comments=True)
    robot = ET.parse(hrdf_file_name, parser).getroot()

    num_actuators = len(list(robot.iter('actuator')))
    
    if args.actuators is None:
        actuator_names = get_names(num_actuators)
    elif len(args.actuators) == num_actuators:
        actuator_names = args.actuators
    else:
        print(args.actuators)
        print(type(args.actuators))
        msg = 'Given {} names, but there are {} actuators in model'.format(len(args.actuators), num_actuators)
        raise ValueError(msg)

    actuator_types = []
    for idx, el in enumerate(robot.iter('actuator')):
        el.set('name', "$(arg prefix)" + actuator_names[idx])
        if 'type' in el.attrib:
            el.set('type', el.attrib['type'].replace('-', '_'))
            actuator_types.append(el.attrib['type'])

    # This strips the J#_ from the front of joint names
    # so brackets/links are named less verbosely
    def get_joint_name(name):
        matchstr = r'J\d_(.*)'
        match = re.search(matchstr, name)
        if match:
            actuator_name = match.group(1)
        else:
            actuator_name = name.split('/')[-1]
        return actuator_name

    elmnts = list(robot)
    for idx, el in enumerate(elmnts):
        if el.tag == 'bracket':
            next_actuator_name = get_joint_name(elmnts[idx+1].attrib['name'])
            el.set('name', '$(arg prefix){}_bracket'.format(next_actuator_name))
        elif el.tag == 'link':
            prev_actuator_name = get_joint_name(elmnts[idx-1].attrib['name'])
            next_actuator_name = get_joint_name(elmnts[idx+1].attrib['name'])
            el.set('name', '$(arg prefix){}_{}'.format(prev_actuator_name, next_actuator_name))
        elif el.tag == 'end-effector':
            el.tag = 'gripper'
            if 'type' not in el.attrib:
                el.set('type', 'Custom')
            el.set('name', '$(arg prefix)end_effector')

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
                    el.set(prop, '${{{}}}'.format(val))

    has_dummy_end = False
    # add a placeholder end effector if chain ends in a non-gripper
    if elmnts[-1].tag != '{'+NS_XACRO+'}gripper':
        has_dummy_end = True
        dummy_end = ET.Element('{'+NS_XACRO+'}gripper', {'type': 'Custom', 'name': 'end_effector', 'mass':'0.005'})
        robot.append(dummy_end)
    elif elmnts[-1].attrib['type'] == 'Custom':
        has_dummy_end = True
        elmnts[-1].set('mass', '0.005')

    # set child names for all elements
    elmnts = list(robot)
    for idx, el in enumerate(elmnts):
        if idx+1 == len(elmnts):
            break
        el.set('child', elmnts[idx+1].attrib['name'])

    robot.tag = ('robot')
    robot.set('name', model_name)

    base_joint = ET.Element('joint', {'name': '$(arg prefix)base_joint', 'type': 'fixed'})
    ET.SubElement(base_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    ET.SubElement(base_joint, 'parent', {'link': '$(arg prefix)base_link'})
    ET.SubElement(base_joint, 'child', {'link': '{}/INPUT_INTERFACE'.format(elmnts[0].attrib["name"])})
    robot.insert(0, base_joint)
    
    robot.insert(0, ET.Element('link', {'name': '$(arg prefix)base_link'}))

    robot.insert(0, ET.Element('{'+NS_XACRO+'}include', {'filename': '$(find hebi_description)/urdf/components/hebi.xacro'}))
    robot.insert(0, ET.Comment(' HEBI {} Arm Kit '.format(model_name)))
    
    robot.insert(0, ET.Element('{'+NS_XACRO+'}arg', {'name': 'prefix', 'default': ''}))

    ET.cleanup_namespaces(robot, top_nsmap={'xacro': NS_XACRO})
    xmlstr = ET.tostring(
        robot,
        pretty_print=True,
        xml_declaration=True,
        with_tail=False,
        encoding='UTF-8'
    )

    outfile = join(args.urdfdir, '{}.urdf.xacro'.format(model_name))
    print('Writing to {}'.format(outfile))
    with open(outfile, 'wb') as f:
        f.write(xmlstr)
