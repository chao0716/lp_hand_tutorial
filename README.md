# LeapHand Tip Control, Frame & IK Tutorial  
*A practical guide for defining frames, understanding transformations, and generating targets for arm, finger, or combined kinematic chains.*

---

## Understanding Frames in URDF

A URDF is essentially a **tree of coordinate systems**:

- Every **link** defines a *frame*.
- Every **joint** defines how one frame moves relative to another (revolute, fixed, prismatic, fixed, etc.).

Typical robotic arm example:

```text
arm_base_link → arm_link_1 → arm_link_2 → arm_end_effector_link
```

The *first link* in a chain is called the **base_link** (or `arm_base_link`, `finger_base_link`, `dummy_link`, etc.).  
All target positions you send to IK are **expressed relative to this base link** unless you explicitly transform them.

In practice for arms/fingers you might have, for example:

```text
arm_base_link → arm_link_1 → arm_link_2 → arm_end_effector_link
finger_base_link → finger_link_1 → finger_link_2 → finger_end_effector_link
```

---

## Defining Kinematic Chains with IKPy

IKPy lets you define a **kinematic chain** by listing the frames you want to include.  
You can define:  
- an **arm-only** chain,  
- a **finger-only** chain, or  
- a **combined arm + finger** chain.

### Arm-only chain

```python
from ikpy.chain import Chain

chain_arm = Chain.from_urdf_file(
    urdf_file=urdf_path,
    base_elements=[
        "arm_base_link",
        "arm_joint_0",
        "arm_link_1",
        "arm_joint_1",
        "arm_link_2",
        "arm_joint_2",
        "arm_end_effector_link",
    ],
    active_links_mask=[False, True, True, False],  # [ikpy_link, arm_joint_0, arm_joint_1, arm_joint_2]
)
```

**Important notes about `active_links_mask`:**

- The **first value is always `False`** (reserved by IKPy).
- Any **fixed joint** in the chain must be set to `False` at its position in the mask.
- In this example, the joint between `arm_link_2` and `arm_end_effector_link` is fixed, so its mask entry is `False`.

---

### Finger-only chain

```python
chain_finger = Chain.from_urdf_file(
    urdf_file=urdf_path,
    base_elements=[
        "finger_base_link",
        "finger_joint_0",
        "finger_link_1",
        "finger_joint_1",
        "finger_link_2",
        "finger_joint_2",
        "finger_end_effector_link",
    ],
    active_links_mask=[False, True, True, False],  # [ikpy_link, finger_joint_0, finger_joint_1, finger_joint_2]
)
```

---

### Combined Arm + Finger chain

If you want **one big chain** including both arm and finger:

```python
chain_arm_finger = Chain.from_urdf_file(
    urdf_file=urdf_path,
    base_elements=[
        "arm_base_link",
        "arm_joint_0",
        "arm_link_1",
        "arm_joint_1",
        "arm_link_2",
        "arm_joint_2",
        "arm_end_effector_link",
        "finger_base_link",
        "finger_joint_0",
        "finger_link_1",
        "finger_joint_1",
        "finger_link_2",
        "finger_joint_2",
        "finger_end_effector_link",
    ],
    active_links_mask=[
        False,  # ikpy_link (internal)
        True,   # arm_joint_0
        True,   # arm_joint_1
        False,  # arm_joint_2 (fixed)
        True,   # finger_joint_0
        True,   # finger_joint_1
        False,  # finger_joint_2 (fixed)
    ],
)
```

---

## Forward Kinematics — Getting Transforms

Forward kinematics (FK) gives the **pose of frames** in the chain.

### Only final end-effector transform

```python
T_ee = chain.forward_kinematics(joint_positions, full_kinematics=False)
```

- Returns a single 4×4 transform matrix:  
  **Transform from `base_link` → `end_effector_link`**.

---

### 3.2 Transform of *each* link in the chain

```python
T_all = chain_arm.forward_kinematics(joint_positions, full_kinematics=True)
```

This returns a **list of 4×4 transforms**. Conceptually:

```python
T_all[0]  # transform of arm_base_link
T_all[1]  # transform of arm_joint_0
T_all[2]  # transform of arm_link_1
T_all[3]  # transform of arm_joint_1
...
```

If index `k` corresponds to `arm_end_effector_link`, then:

```python
T_base_armEE = T_all[k]   # transform of arm_end_effector_link relative to arm_base_link
```

The key idea:  
> `full_kinematics=True` gives you access to **all intermediate frames**, not just the final end-effector.

You must know (or inspect) which **index** in `T_all` corresponds to which link.

---

## Basic IK Examples

### Just moving the finger (target relative to `finger_base_link`)

```python
import numpy as np

target = np.array([0.653, 0.51, 1.032])  # expressed in finger_base_link frame

joint_angles = chain_finger.inverse_kinematics(
    target_position=target,
    initial_position=initial_position,
)
```

Here:

- `target` is a 3D point **relative to `finger_base_link`**,  
- IK returns joint angles that place `finger_end_effector_link` at that point.

---

## Targets Relative to Different Frames

Often you want to express motion in a **local frame** (e.g., “move 2 cm in the local y-axis of the arm end-effector”). To do this, you:

1. Get the transform of the frame (via FK),
2. Build a small local offset (`T_delta`),
3. Multiply to get the **global target** matrix.

### Arm + Finger — target relative to `arm_end_effector_link`

Example: move arm+finger so that **finger end-effector** reaches a point that is defined relative to `arm_end_effector_link`.

```python
# 1) Compute all transforms in the chain
T_all = chain_arm_finger.forward_kinematics(initial_position, full_kinematics=True)

# Suppose index 3 corresponds to arm_end_effector_link
T_base_armEE = T_all[3]   # transform arm_end_effector_link relative to arm_base_link

# 2) Create a local offset in the arm EE frame
T_delta = np.eye(4)
T_delta[:3, 3] = np.array([0.00, 0.02, 0.00])   # 2 cm in local y-axis of arm_end_effector_link

# 3) Compute the NEW target in base_link coordinates
T_target = T_base_armEE @ T_delta
```

`T_target` is now the **target pose expressed in the base_link frame**, which you can pass to an IK solver that expects a pose relative to `base_link`.

---

### Arm + Finger — target relative to `finger_end_effector_link`

Now imagine: “move the finger itself 2 cm in its own local Y axis”.

1. Get the **current finger end-effector pose** (only final transform needed):

```python
T_fingerEE = chain_arm_finger.forward_kinematics(
    updated_position, full_kinematics=False
)

T_delta = np.eye(4)
T_delta[:3, 3] = np.array([0.00, 0.02, 0.00])   # 2 cm in local y-axis of finger_end_effector_link

T_target = T_fingerEE @ T_delta
```

The pattern is always:

```text
T_target_global = T_current_global @ T_local_offset
```

Where `T_local_offset` encodes the small motion in the **local frame**.

---

## Choosing the Right Chain

The selected chain defines **what is allowed to move** to satisfy your IK target.

| What you want to move        | Chain to use        |
|------------------------------|---------------------|
| Only the arm                 | `chain_arm`         |
| Only the finger              | `chain_finger`      |
| Arm and finger together      | `chain_arm_finger`  |

Examples:

- **Only arm**: use `chain_arm.inverse_kinematics(...)`  
- **Only finger**: use `chain_finger.inverse_kinematics(...)`  
- **Arm + finger**: use `chain_arm_finger.inverse_kinematics(...)`

---

## Practical Notes (URDF Examples)

Two URDFs are relevant here:

- `leap_hand.urdf`
- `robot.urdf`

The only conceptual difference:

- In `robot.urdf` a **custom base_link** is defined at a location that is convenient for targeting (e.g., mounting point on a robot, or a clean reference for the hand).

This lets you choose where it is **most natural** to define your targets, without changing the rest of the kinematic chain.

---

## Summary

- Every **link** in URDF = a **coordinate frame**.
- Joints describe how frames are connected (and whether they move or are fixed).
- IK targets are always **relative to the chain’s base_link**.
- Use `forward_kinematics(..., full_kinematics=True)` when you need transforms for **intermediate frames**.
- Use `forward_kinematics(..., full_kinematics=False)` when you only need the **final end-effector** transform.
- To move “in a local frame” (arm EE, finger EE, etc.), you:
  1. Get the frame’s transform with FK.
  2. Build a local offset `T_delta`.
  3. Compute `T_target = T_frame @ T_delta`.
  4. Extract `T_target` into position + orientation and feed to IK.

With this, you can:

- Drive only the finger, only the arm, or both.
- Express targets in **any frame** in the chain, then convert them back to the appropriate base_link.
- Keep a clean mental model of your frames and transformations, which is crucial for stable tip control.

---

# Practical MuJoCo and Isaac examples

## MuJoCo

A concise walkthrough to convert a URDF to MuJoCo (MJCF), validate a tool reference frame, and test tip-control with included scripts.

### Overview
This repo demonstrates:
- Optional: add a tool link/joint to URDF for a custom end-effector reference
- Convert URDF -> MJCF
- STEP 1: fix the generated XML so it doesn't fall in MuJoCo
- Validate the tool frame (in MuJoCo viewer)
- STEP 2: add actuators/sensors for tip-control
- Test control using the provided scripts

### Prerequisites
Install dependencies (example):
```bash
sudo apt install python3.11 python3.11-venv
python3.11 -m venv tip_control_venv
source tip_control_venv/bin/activate

cd ~/Leap-hand-tipcontrol-tutorial/
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### Optional: URDF tool frame (do this before conversion)
Add a fixed link & joint in your URDF if you want a dedicated tool frame (e.g., link `fingertip_end` and fixed joint id `4`). This gives a predictable reference to target from the end-effector.

### Convert URDF to MJCF
Convert the URDF with:
```bash
cd ~/Leap-hand-tipcontrol-tutorial/
urdf2mjcf path/to/.urdf --output path/to/.xml
# In this example
urdf2mjcf ./urdf/robot.urdf --output xml/robot.xml
```
This writes the MuJoCo XML to `xml/robot.xml`.

### STEP 1 — Fix the generated XML (mandatory)
Open `xml/robot.xml` and follow STEP 1 from `robot_template.xml` in this repo:
- Fix base links to avoid falling (e.g., set freejoint/floating base to fixed or add constraints).
- Fix initial poses or inertias if needed.

Make these STEP 1 changes before running the viewer.

### Validate the reference frame in MuJoCo (after STEP 1)
Open the viewer:
```bash
cd ~/Leap-hand-tipcontrol-tutorial/
python view_mjcf.py path/to/.xml
# In this example
python view_mjcf.py xml/robot_template.xml
```
In MuJoCo Viewer:
- Set "Rendering" -> "Label" and "Frame" to "Body" to inspect frames and names.
- Verify the tool frame (e.g., `fingertip_end`) is where you expect it.

Screenshot (visual aid):
![MuJoCo rendering](assets/MuJoCo_rendering.png)

If the frame is misaligned, adjust the URDF/tool link and re-run convert + STEP 1.

### STEP 2 — Add actuators, sensors and control bodies
Once STEP 1 validation is OK:
- Make STEP 2 edits in `xml/robot.xml` following `robot_template.xml`.
- STEP 2 adds positions controlled actuators/actuator and their mappings, a `marker` and a `target_marker` body, and any sensor or geom markers needed for control.

After STEP 2, test manual control with the Control tab (sliders) in the viewer.

### Validate actuators
Open the viewer:
```bash
cd ~/Leap-hand-tipcontrol-tutorial/
python view_mjcf.py path/to/.xml
# In this example
python view_mjcf.py xml/robot_template.xml
```

### Test control with the script
Run the controller that uses IK or direct control to move the tip:
```bash
cd ~/Leap-hand-tipcontrol-tutorial/

# Use defaults
python control.py

# Or specify URDF and XML paths explicitly:
python control.py --urdf /path/to/.urdf --xml /path/to/.xml

# In this case
python control.py --urdf ./urdf/robot.urdf --xml ./xml/robot_template.xml
```
Notes:
- `control.py` uses ikpy (or your chosen IK) and sends targets relative to last link or your `fingertip_end` link.
- Adjust your target points to match the chosen reference frame.

Rendering tips: set rendering to show labels and frames; the viewer markers:
- `marker` (yellow) — the reference frame at the end-effector
- `target_marker` (light blue) — the commanded target position

Screenshot (visual aid):
![Rendering for control confirmation](assets/MuJoCo_rendering_2.png)

### Quick commands summary
```bash
# Optional: Add a tool/frame to your URDF (do this BEFORE converting)
# (e.g., add a fixed link 'fingertip_end' and a fixed joint so you have a predictable end-effector frame)

# 1. Convert URDF -> MJCF
urdf2mjcf /path/to/.urdf --output /path/to/.xml

# 2. STEP 1: Edit xml/robot.xml to stabilize/fix the base
# Make STEP 1 changes from robot_template.xml (prevent falling/spinning)

# 3. Validate frames and body names in MuJoCo Viewer (AFTER STEP 1)
python view_mjcf.py /path/to/.xml
# In Viewer: set Rendering -> Label and Frame to 'Body' and inspect the tool frame
# See assets/MuJoCo_rendering.png for suggested viewer settings

# 4. If needed: adjust URDF/tool link, re-convert and re-apply STEP 1

# 5. STEP 2: Edit xml/robot.xml and add actuators/targets/sensor bodies
# Apply STEP 2 changes from robot_template.xml (target_marker, actuators, etc.)

# 6. Validate actuators and manual control in the Viewer (Control tab sliders)
python view_mjcf.py /path/to/.xml
# Confirm 'marker' (yellow) and 'target_marker' (light blue) are present

# 7. Run programmatic tip control (specify URDF/XML if not using defaults)
python control.py --urdf /path/to/.urdf --xml /path/to/.xml
```

### Troubleshooting
- Robot falls/spins: re-check STEP 1 edits (base constraints).
- Markers missing: check site definitions in the XML.
- Control not working: check actuator and default class definitions

### Files of interest
- `xml/robot.xml` — generated MJCF (apply STEP 1 & STEP 2)
- `robot_template.xml` — example STEP 1 & STEP 2 edits
- `view_mjcf.py` — opens MuJoCo viewer


## Isaac Sim

A walkthrough to convert a URDF to USD and control the end-effector relative to the base.

### How it works

The script node computes joint angles as outputs for the articulation controller node in Isaac Action Graph. When the articulation controller receives the positions, it moves the arm accordingly..

### Setup
Download Isaac Sim 5.1 following the [installation guide](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html).

Install the IK dependency:
```bash
~/isaacsim/python.sh -m pip install ikpy
```
### Convert URDF to USD
Follow the [URDF import guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/ext_isaacsim_asset_importer_urdf.html) to convert your own URDF to USD format.

### Configuration
1. Open `action_graph_script.py` and set the absolute path to your URDF (see code comments).
2. Start the example simulation `leap_hand.usda`.

### Demo
**[Demonstartion Video](assets/isaac_demo.webm)**
