#!/usr/bin/env python3
from pyexpat import model
import sys
import time
import numpy as np
import argparse
import os

import mujoco
import mujoco.viewer

from ikpy.chain import Chain

MARKER_NAME = "marker"
TARGET_MARKER_NAME = "target_marker"

# Determine base directory and default file paths
this_dir = os.path.dirname(os.path.abspath(__file__))
default_urdf = os.path.join(this_dir, "urdf", "robot.urdf")
default_xml = os.path.join(this_dir, "xml", "robot_template.xml")

# Parse CLI arguments for file paths
parser = argparse.ArgumentParser(description="Tip-control demo using MuJoCo & ikpy")
parser.add_argument("--urdf", default=default_urdf, help="Path to the URDF file (default: %(default)s)")
parser.add_argument("--xml", default=default_xml, help="Path to the MuJoCo XML (MJCF) file (default: %(default)s)")
args = parser.parse_args()

# Use paths provided by the user or the defaults
urdf_path = args.urdf
xml_path = args.xml

def generate_ikpy_chain(urdf_path):
    """
    Build the IK chain from your URDF file.

    For IKPy to work links and joints must be provided manually in correct order.
    IMPORTANT: The last element must be the end-effector link which is the reference for IK targets. 

    base_elements:
        A list describing the ordered sequence of links/joints from
        base_link → fingertip_end.

    active_links_mask:
        A Boolean list marking which elements of the chain are actuated.
        Length must match the number of joints in IKPy's internal chain.

        In this example:
            Index 0  → a dummy fixed joint in IKPy   (False)
            Index 1  → joint "1"    (True)
            Index 2  → joint "0"    (True)
            Index 3  → joint "2"    (True)
            Index 4  → joint "3"    (True)
            Index 5  → fingertip_end (False, because it's not a joint)

    Returns:
        An ikpy.Chain object usable for IK and FK.
    """
    return Chain.from_urdf_file(
        urdf_file=urdf_path,
        base_elements=[
            "base_link",
            "1",
            "mcp_joint",
            "0",
            "pip",
            "2",
            "dip",
            "3",
            "fingertip",
            "4",
            "fingertip_end",
        ],
        active_links_mask=[False, True, True, True, True, False],
    )

def main(urdf_path, xml_path):

    # --- 1. Load model and create data --------------------------------------
    # MjModel: contains all *static* info about the model (joints, bodies, actuators,...)
    model = mujoco.MjModel.from_xml_path(xml_path)

    # MjData: contains all *dynamic* state (qpos, qvel, ctrl, forces,...)
    data = mujoco.MjData(model)

    # --- 2. Map actuator names from XML to indices in data.ctrl -------------
    actuator_names = ["1_ctrl", "0_ctrl", "2_ctrl", "3_ctrl"]

    # actuator_ids, in this case, will be {"1_ctrl": 0, "0_ctrl": 1, "2_ctrl": 2, "3_ctrl": 3}
    actuator_ids = {}
    for name in actuator_names:
        # model.actuator(name) gives a handle to that actuator; .id is its index
        aid = model.actuator(name).id
        actuator_ids[name] = aid
        print(f"Actuator '{name}' has id={aid}")

    # Build IK chain
    chain = generate_ikpy_chain(urdf_path)

    # IKPy always expects a full list of joint angles for ALL joints
    # in the chain (including inactive/fixed ones).
    #
    # Our chain has 6 joints in total due to the active_links_mask,
    # even though only 4 are actuated.
    #
    # IMPORTANT:
    #   IKPy joint alignment = IKPy internal chain
    #   MuJoCo actuator alignment = actuator order in XML
    #
    # They are not guaranteed to match.
    # That is why we manually map:
    #
    #   IKPy joint index → actuator name → data.ctrl
    #
    # Only indices [1,2,3,4] belong to actuated joints.
    # -------------------------------------------------------
    initial_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # This will be updated each iteration with the last IK solution.
    updated_position = None

    # --- Open the viewer --------------------------------------------------
    # launch_passive means the viewer window does not itself drive the simulation.
    # We are responsible for calling mj_step() and setting data.ctrl.

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Main simulation loop: runs until the viewer window is closed
        while viewer.is_running():
            # First loop: copy initial positions
            if updated_position is None:
                updated_position = initial_position.copy()
            
                # 1) Current EE world pose
                T_ee = chain.forward_kinematics(updated_position, full_kinematics=False)
                # 2) Create small motion in EE local frame
                T_delta = np.eye(4)
                T_delta[:3, 3] = np.array([0.00, 0.02, 0.00])   # desired local EE movement (x, y, z) axis

                # 3) Compute NEW target in world coordinates
                T_target = T_ee @ T_delta
    
                # 4) Extract IKPy inputs (world coordinates)
                target_position = T_target[:3, 3]
                target_orientation = T_target[:3, :3]

                # Retrieve the site id of the marker from xml
                marker_sid = model.site(MARKER_NAME).id

                # Move MuJoCo marker to starting position of EE
                model.site_pos[marker_sid] = np.array(T_ee[:3, 3])        

                # Retrieve the site id of the target marker from xml
                target_marker_sid = model.site(TARGET_MARKER_NAME).id          

                # Move MuJoCo marker to target position
                model.site_pos[target_marker_sid] = np.array(target_position)

            # --- Calculate IK -------
            joint_angles = chain.inverse_kinematics(
                target_position=target_position, 
                # target_orientation=target_orientation,
                # orientation_mode="X",
                initial_position=updated_position
            )
            
            # Update initial position for next iteration
            #     updated_position[1] = joint  "1"
            #     updated_position[2] = joint  "0"
            #     updated_position[3] = joint  "2"
            #     updated_position[4] = joint  "3"
            updated_position[1:5] = joint_angles[1:5]

            # Collect into a dict indexed by actuator name
            joint_positions = {
                "1_ctrl": joint_angles[1],
                "0_ctrl": joint_angles[2],
                "2_ctrl": joint_angles[3],
                "3_ctrl": joint_angles[4],
            }

            # --- Write those target positions into data.ctrl --------------
            #
            # For each actuator name -> id, set the corresponding control entry.
            # This is the key step: this is how you actually "command" position
            # to each joint in a position-controlled actuator.
            for name, aid in actuator_ids.items():
                data.ctrl[aid] = joint_positions[name]

            # --- Step the simulation ---------------------------------------
            # mj_step integrates one timestep: qpos, qvel, martkers, etc. are updated.
            mujoco.mj_step(model, data)

            # Update the viewer with the new state
            viewer.sync()


if __name__ == "__main__":
    main(urdf_path, xml_path)
