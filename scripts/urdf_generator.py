#! /usr/bin/env python3

import argparse
import sys
import os
from os.path import basename, splitext, join
import subprocess
import re
import numpy as np
from scipy.spatial.transform import Rotation as R

from lxml import etree as ET

NS_XACRO = 'http://wiki.ros.org/xacro'
ET.register_namespace('xacro', NS_XACRO)


def get_names(num_names):
    names = []
    print("There are {} actuators in the provided model. Please provide names for each.".format(num_names))
    for i in range(num_names):
        names.append(input('Input Actuator {} Name: '.format(i+1)))
    return names

def simplify_rot_expr(expression):
    # Remove any whitespace for easier parsing
    expression = expression.replace(" ", "")
    # Find all rotation functions and their arguments
    rotations = re.findall(r'([Rxyz]+)\(([^)]+)\)', expression)

    final_rotmat = np.eye(3)

    for axis, expr in rotations:
        # Replace pi with np.pi
        expr = expr.replace("pi", "np.pi")
        expr = eval(expr)
        if axis == "Rx":
            rot_mat = np.array([[1, 0, 0],
                                [0, np.cos(expr), -np.sin(expr)],
                                [0, np.sin(expr), np.cos(expr)]])
        elif axis == "Ry":
            rot_mat = np.array([[np.cos(expr), 0, np.sin(expr)],
                                [0, 1, 0],
                                [-np.sin(expr), 0, np.cos(expr)]])
        elif axis == "Rz":
            rot_mat = np.array([[np.cos(expr), -np.sin(expr), 0],
                                [np.sin(expr), np.cos(expr), 0],
                                [0, 0, 1]])
        else:
            raise ValueError(f"Invalid rotation axis: {axis}")

        final_rotmat = final_rotmat @ rot_mat
    
    # Convert rotation matrix to euler angles
    euler = R.from_matrix(final_rotmat).as_euler('xyz')
    euler_str = " ".join([f"{round(e, 4):g}" for e in euler])
    return euler_str

def read_hrdf(hrdf_file_name):
    parser = ET.XMLParser(remove_blank_text=True, remove_comments=True)
    return ET.parse(hrdf_file_name, parser).getroot()

# Print in hierarchical order and level-based tabbing
def print_robot(robot):
    def print_recursive(el, level):
        print("    "*level + "@" + el.tag)
        for key, val in el.attrib.items():
            print("    "*level + "- " + key + ": " + val)
        for child in el:
            print_recursive(child, level+1)

    return print_recursive(robot, 0)

def flatten_etree(tree):
    root = list(tree.iter())[0]
    new_root = ET.Element(root.tag, attrib=root.attrib)
    
    for element in root.iter():
        if element.tag == root.tag:
            continue
        new_element = ET.Element(element.tag, attrib=element.attrib)
        new_root.append(new_element)

    return new_root

def convert_to_URDF(hrdf_file_name, meshdir):
    model_name = splitext(basename(hrdf_file_name))[0]
    robot = read_hrdf(hrdf_file_name)

    if len(list(robot.iter('robot'))) > 1:
        raise ValueError("Multiple robot tags found in HRDF file")
    # Joint is not supported yet
    if len(list(robot.iter('joint'))) > 0:
        raise ValueError("Joint tag not supported yet")

    for include in robot.iter('include'):
        include_fp = include.attrib['path']
        if include_fp == basename(hrdf_file_name):
            raise ValueError(f"Recursive include detected: Include path {include_fp} is same as current file")
        
        include_fp = join(os.path.dirname(hrdf_file_name), include_fp)
        include_root = read_hrdf(include_fp)
        include_content = include_root.getchildren()
        
        # Replace include tag with all the included content
        parent = include.getparent()
        for el in include_content:
            parent.append(el)
        parent.remove(include)

    # Simplify rotation expressions
    for el in robot.iter():
        for key, val in el.attrib.items():
            if "rot" in key:
                el.attrib[key] = simplify_rot_expr(val)

    # Name all rigid bodies and check for mesh files
    for idx, el in enumerate(robot.iter('rigid-body')):
        el.attrib['name'] = f"RB{idx+1}"
        if 'mesh_path' in el.attrib:
            # check if mesh_path is an URL
            if el.attrib['mesh_path'].startswith('http'):
                print("URL detected in mesh_path: ", el.attrib['mesh_path'])
                filename = el.attrib['mesh_path'].split('/')[-1]
                filepath = join(meshdir, filename)
                # Check if file already exists
                if not os.path.exists(filepath):
                    print(f"Downloading mesh file to {meshdir}")
                    subprocess.run(['wget', '-O', join(meshdir, filename), el.attrib['mesh_path']])
                else:
                    print(f"Mesh file {filename} already exists in {meshdir}")
                el.attrib['mesh_path'] = "file://" + join(meshdir, filename)
            elif el.attrib['mesh_path'] != "":
                el.attrib['mesh_path'] = "file://" + join(meshdir, el.attrib['mesh_path'])
            else:
                el.attrib.pop('mesh_path', None)

    # Name all actuators and joints
    for idx, el in enumerate(robot.iter('actuator', 'joint')):
        el.attrib['name'] = f"J{idx+1}"
        if el.tag == 'actuator':
            el.attrib['type'] = el.attrib['type'].replace('-', '_')

    # Name all links
    for idx, el in enumerate(robot.iter('link')):
        el.attrib['name'] = f"L{idx+1}"

    # Name all brackets
    for idx, el in enumerate(robot.iter('bracket')):
        el.attrib['name'] = f"B{idx+1}"
    
    # Name all end effectors
    for idx, el in enumerate(robot.iter('end-effector')):
        el.attrib['name'] = f"EE{idx+1}"
        el.tag = 'gripper'
        if 'type' not in el.attrib:
            el.set('type', 'Custom')
        if 'mass' not in el.attrib:
            el.set('mass', '0.01')

    # Rigid bodies and brackets
    for el in robot.iter('rigid-body', 'bracket'):
        if el.tag == 'rigid-body' and el.attrib['mass'] == '0':
            el.attrib['mass'] = '0.01'
        # If rigid body or bracket has zero children, make an output tag inside
        if len(list(el)) == 0:
            output_el = ET.Element('output')
            # Move all elements after the element inside the output tag
            for sibling in el.getparent()[el.getparent().index(el)+1:]:
                output_el.append(sibling)
            if len(list(output_el)) > 0:
                if el.tag == 'bracket':
                    output_el.attrib['type'] = el.attrib['type']
                else:
                    if 'output_trans' in el.attrib:
                        output_el.attrib['trans'] = el.attrib['output_trans'] 
                    if 'output_rot' in el.attrib:
                        output_el.attrib['rot'] = el.attrib['output_rot']
                el.append(output_el)
        else:
            # Set trans and rot attributes for all children from parent or set to 0 0 0
            for child in el:
                if el.tag == 'bracket':
                    child.attrib['type'] = el.attrib['type']
                    child.attrib.pop('trans', None)
                    child.attrib.pop('rot', None)
                    continue
                if 'trans' not in child.attrib and 'output_trans' in el.attrib:
                    child.attrib['trans'] = el.attrib['output_trans']
                if 'rot' not in child.attrib and 'output_rot' in el.attrib:
                    child.attrib['rot'] = el.attrib['output_rot']
            
        # Remove output_trans and output_rot attributes from parent
        el.attrib.pop('output_trans', None)
        el.attrib.pop('output_rot', None)
    
    name_counter = 1
    for el in robot.iter('output'):    
        if len(list(el)) == 0:
            # Make dummy link inside output
            link_el = ET.Element('dummy-link', {'name': f"output_link{name_counter}"})
            name_counter += 1
            el.append(link_el)
        
        parent_name = el.getparent().attrib['name']
        el.attrib['parent'] = parent_name
        child_name = el[0].attrib['name']
        el.attrib['child'] = child_name
        el.attrib['name'] = f"{parent_name}_{child_name}"
    
    for el in robot.iter('actuator', 'link'):
        el.attrib['child'] = list(el.getparent())[el.getparent().index(el)+1].attrib['name']
        if el.tag == 'link' and 'output' not in el.attrib:
            el.attrib['output'] = "None" if "EE" in el.attrib['child'] else "RightAngle"
    
    robot = flatten_etree(robot)
    
    for idx, el in enumerate(robot.iter()):
        if el.tag == 'include':
            el.attrib.pop('name', None)
        if 'name' in el.attrib:
            el.attrib['name'] = "$(arg prefix)" + el.attrib['name']
        if 'child' in el.attrib:
            el.attrib['child'] = "$(arg prefix)" + el.attrib['child']
        if 'parent' in el.attrib:
            el.attrib['parent'] = "$(arg prefix)" + el.attrib['parent']

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
        
        # Handle dummy links
        if el.tag == '{'+NS_XACRO+'}dummy-link':
            el.tag = 'link'

    robot.tag = ('robot')
    robot.set('name', model_name)
    
    base_joint = ET.Element('joint', {'name': '$(arg prefix)base_joint', 'type': 'fixed'})
    base_xyz = "0 0 0" if 'trans' not in robot.attrib else robot.attrib['trans']
    base_rpy = "0 0 0" if 'rot' not in robot.attrib else robot.attrib['rot']
    ET.SubElement(base_joint, 'origin', {'xyz': base_xyz, 'rpy': base_rpy})
    ET.SubElement(base_joint, 'parent', {'link': '$(arg prefix)base_link'})
    ET.SubElement(base_joint, 'child', {'link': list(robot.iter())[1].attrib['name']})
    robot.insert(0, base_joint)
    
    robot.insert(0, ET.Element('link', {'name': '$(arg prefix)base_link'}))

    robot.insert(0, ET.Element('{'+NS_XACRO+'}include', {'filename': '$(find hebi_description)/urdf/components/hebi.xacro'}))
    robot.insert(0, ET.Comment(' HEBI {} Arm Kit '.format(model_name)))
    
    robot.insert(0, ET.Element('{'+NS_XACRO+'}arg', {'name': 'prefix', 'default': ''}))

    robot.attrib.pop('version', None)

    ET.cleanup_namespaces(robot, top_nsmap={'xacro': NS_XACRO})
    xmlstr = ET.tostring(
        robot,
        pretty_print=True,
        xml_declaration=True,
        with_tail=False,
        encoding='UTF-8'
    )

    outfile = join(os.path.dirname(hrdf_file_name), '{}.urdf.xacro'.format(model_name))
    print('Writing to {}'.format(outfile))
    with open(outfile, 'wb') as f:
        f.write(xmlstr)
        

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Convert HRDF files into xacro/sdf equivalent.')
    parser.add_argument('filename')
    parser.add_argument('--actuators', nargs="+", type=str, default=None)
    parser.add_argument('--meshdir', default='meshes')

    args = parser.parse_args()
    hrdf_file_name = args.filename
    meshdir = args.meshdir
    # If mesh dir is relative, make it absolute
    if not os.path.isabs(meshdir):
        meshdir = os.path.abspath(meshdir)

    convert_to_URDF(hrdf_file_name, meshdir)

    exit()