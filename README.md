# hebi_description

This repository contains a collection of **Xacro macros** for generating **URDF files** tailored to HEBI components and systems. These macros enable modular and reusable robot descriptions, including predefined macros for complete systems.

Before proceeding, it is recommended to familiarize yourself with the **HRDF format**, which you can find [here](https://github.com/HebiRobotics/hebi-hrdf/blob/main/FORMAT.md).

## Limitations

Currently, HEBI `<joint>` tags and the following optional attributes are **not supported** for `actuator`, `link`, `bracket`, and `end-effector`:
- `mass_offset`
- `com_trans_offset`
- `mass`
- `com_rot`
- `com_trans`
- `ixx`, `iyy`, `izz`, `ixy`, `ixz`, `iyz`

## HRDF to URDF Conversion

A script is provided to convert **HRDF files** into **URDF Xacro** format at `scripts/urdf_generator.py`

### Usage
```
python3 scripts/urdf_generator.py [-h] [--actuators ACTUATORS [ACTUATORS ...]] [--meshdir MESHDIR] [--outputdir OUTPUTDIR] filename
```
- `filename`: Path to an HRDF file or a HEBI Config file.
- The script extracts actuator names directly from the config file and processes the HRDF file path.

### Dependencies
Ensure the following Python libraries are installed:
- `hebi-py`
- `numpy`
- `scipy`
- `lxml`

Install dependencies via:
```
pip3 install --user hebi-py numpy scipy lxml
```

For more details on using Xacro macros, refer to the [ROS Wiki](http://wiki.ros.org/xacro).

## HEBI Xacro Macros

This repository provides several `xacro` macros for HEBI components that can be used to create robots for simulation or visualization.

**Note**: The mesh files in the Xacros are scaled by default to 0.001 in the the HEBI component `xacro` macros.

### `<xacro:actuator/>`

Represents a HEBI actuator.

**Required attributes:**
- `name`: Unique identifier for referencing this actuator.
- `child`: The element attached to the actuator's output (name of the HEBI component).
- `type`: Actuator type (`X5_1`, `X5_4`, `X5_9`, `X8_3`, `X8_9`, or `X8_16`).

**Note:** Since the actuator is a **joint**, the `name` is set for the `<joint>` tag and the actuator link name is set as `<name>/body`.

**Optional attribute:**
- `limits`: Position limits in radians. Format: `${[<low>,<high>]}`. Example: `${[-${pi}, ${pi}]}` for one full revolution centered at 0. If not defined, assumes a continuously rotatable revolute joint.

### `<xacro:link/>`

**Required attributes:**
- `name`: Unique identifier for referencing this link.
- `child`: The element attached to the link's output.
- `extension`: Link extension in meters, per documented convention on docs.hebi.us.
- `twist`: Twist between input and output frames in radians, per documented convention on docs.hebi.us.

### `<xacro:bracket/>`

**Required attributes:**
- `name`: Unique identifier for referencing this bracket.
- `type`: Bracket type (`X5LightLeft`, `X5LightRight`, `X5HeavyLeftInside`, `X5HeavyLeftOutside`, `X5HeavyRightInside`, or `X5HeavyRightOutside`).

**Note:** Brackets are always followed by `<xacro:output>` tags for connecting to other components.

### `<xacro:output/>`

**Required attributes:**
- `name`: Unique identifier for referencing this output.
- `parent`: The parent element of the URDF joint.
- `child`: The child element of the URDF joint.

**Optional attribute:**
- `type`: Same as the bracket type if preceded by a bracket. If provided, the properties of the joint will be loaded accordingly.

### `<xacro:rigid_body/>`

Same properties as HRDF `<rigid-body>` tag, with an additional property `mesh_scale` to set the scaling factor of the mesh. Defaults to `0.001 0.001 0.001`.

### `<xacro:gripper/>`

Represents a HEBI gripper (currently only parallel jaws style).

**Required attributes:**
- `name`: Unique identifier for referencing this gripper.
- `type`: Gripper type (currently only `parallel` is supported).

### `<xacro:null_end_effector/>`

Represents the end of a robot (allows the final joint in another `actuator`, `link`, or `bracket` to be completed).

**Required attribute:**
- `name`: Unique identifier for referencing this end effector.