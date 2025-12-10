# Practical MuJoCo visualization and Isaac examples

## MuJoCo

A concise walkthrough to convert a URDF to MuJoCo (MJCF), validate URDF frames.

### Overview
- Optional: add a link/joint to URDF for a custom frame
- Convert URDF -> MJCF
- STEP 1: fix the generated XML so it doesn't fall in MuJoCo
- Validate the frame (in MuJoCo viewer)
- STEP 2: add actuators/sensors for tip-control

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

### Optional: Add URDF frames (do this before conversion)
Add a fixed link & joint in your URDF if you want a dedicated frame (e.g., link `fingertip_end` and fixed joint id `4`). This gives a predictable reference to target from the end-effector.

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
python scripts/view_mjcf.py path/to/.xml
# In this example
python scripts/view_mjcf.py xml/robot_template.xml
```
In MuJoCo Viewer:
- Set "Rendering" -> "Label" and "Frame" to "Site" to inspect frames and names.
- Optionally change the frame length or width in "Visualization" -> "Scale".
- Verify the frame (e.g., `fingertip_end`) is where you expect it.

Screenshot (visual aid):
![MuJoCo rendering](assets/MuJoCo_rendering.png)

If the frame is misaligned, adjust the URDF/tool link and re-run convert + STEP 1.

### STEP 2 — Add actuators, sensors and control bodies
Once STEP 1 validation is OK:
- Make STEP 2 edits in `xml/robot.xml` following `robot_template.xml`.
- STEP 2 adds positions controlled actuators/actuator and their mappings, and any sensor or geom markers needed for control.

After STEP 2, test manual control with the Control tab (sliders) in the viewer.

### Files of interest
- `xml/leap_hand.xml` — generated MJCF (apply STEP 1 & STEP 2)
- `xml/robot_template.xml` — example STEP 1 & STEP 2 edits
- `scripts/view_mjcf.py` — opens MuJoCo viewer

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
3. ikpy implementation instructions `scripts/README.md`

### Demo
**[Demonstartion Video](assets/isaac_demo.webm)**
