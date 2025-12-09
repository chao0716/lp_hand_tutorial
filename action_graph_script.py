# This script is executed the first time the script node computes, or the next time
# it computes after this script is modified or the 'Reset' button is pressed.
#
# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed
# Available variables:
#    db: og.Database The node interface - attributes are exposed in a namespace like db.inputs.foo and db.outputs.bar.
#                    Use db.log_error, db.log_warning to report problems in the compute function.
#    og: The omni.graph.core module

from isaacsim.core.prims import Articulation
from isaacsim.core.utils.prims import get_prim_at_path
from ikpy.chain import Chain

import numpy as np
import os

from pxr import Usd, UsdGeom, Gf
from omni.isaac.core import World
import omni

world = World.instance()
stage = omni.usd.get_context().get_stage()

# absolute path to your URDF file
urdf_path = "/home/milos/Leap-hand-tipcontrol-tutorial/urdf/leap_hand.urdf"

def generate_ikpy_chain(urdf_path):
    """
    Build the IK chain from your URDF file.

    For IKPy to work links and joints must be provided manually in correct order.. 

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
            "joint_1",
            "mcp_joint",
            "joint_0",
            "pip",
            "joint_2",
            "dip",
            "joint_3",
            "fingertip",
            "joint_4",
            "fingertip_end",
        ],
        active_links_mask=[False, True, True, True, True, False],
    )

def get_translation(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    properties = prim.GetPropertyNames()
    if "xformOp:translate" in properties:
        translate_attr = prim.GetAttribute("xformOp:translate")
        return translate_attr.Get()
    elif "xformOp:translation" in properties:
        translation_attr = prim.GetAttribute("xformOp:translation")
        return translation_attr.Get()
    elif "xformOp:transform" in properties:
        transform_attr = prim.GetAttribute("xformOp:transform")
        matrix = prim.GetAttribute("xformOp:transform").Get()
        return matrix.ExtractTranslation()

def generate_joints_map(robot_path):
    joints_map = {}
    # Get all dof paths
    articulation = Articulation(robot_path)
    articulation.initialize()
    dof_paths = articulation._dof_paths[0]
    # Iterate over all dof paths
    for i in range(len(dof_paths)):
        # Get the joint name
        joint_prim = get_prim_at_path(dof_paths[i])
        joint_name = joint_prim.GetName()
        joints_map[joint_name] = int(i)
    return joints_map

def setup(db: og.Database):
    # Get input attributes
    robot_path = str(db.inputs.robotPrim[0].GetPrimPath())
    # Generate the joint mapping of the model (joint_name : index)
    joints_map = generate_joints_map(robot_path)
    # Store in the database
    db.per_instance_state.robot_path = robot_path
    db.per_instance_state.joints_map = joints_map
    db.per_instance_state.chain = generate_ikpy_chain(urdf_path)
    db.per_instance_state.updated_position = None
    # Set initialized to True
    db.per_instance_state.initialized = True


def cleanup(db: og.Database):
    db.outputs.robotPath = ""
    db.outputs.jointIndices = []
    db.outputs.positionCommand = []

def compute(db: og.Database):
    # Ensure shared variables are initialized
    if not db.per_instance_state.initialized:
        cleanup(db)  # Cleanup before reinitializing
        setup(db)    # Initialize shared variables

    # Get the robot path and joints map
    robot_path = db.per_instance_state.robot_path
    joints_map = db.per_instance_state.joints_map
    chain = db.per_instance_state.chain
    
    db.log_info("==== JOINTS_MAP ====")
    # joint_name : index
    db.log_info(str(joints_map))


    target = get_translation("/World/leap_hand/dumay_link/dumay_link_2/base_link/target_frame") # target recieved from the target_frame in isaac sim
    db.log_error(f"Base link translation: {target}")
    
    # target = np.array([0.053, 0.11, -0.032]) # for makin a custom target reference to the base_link

    if db.per_instance_state.updated_position is None:
        updated_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # first iteration
    else:
        updated_position = db.per_instance_state.updated_position  # later iterations with the robot's last known joint position

    joint_angles = chain.inverse_kinematics(
        target_position=target, 
        # target_orientation=target_orientation,
        # orientation_mode="X",
        initial_position=updated_position
    )
    position_command = joint_angles[1:5].tolist() # positions for the joints that are passed to isaac sim

    db.per_instance_state.updated_position = joint_angles.tolist() # store the last known joint positions

    joint_names  = list(joints_map.keys())
    joint_indices = list(joints_map.values())

    db.outputs.robotPath = robot_path
    db.outputs.jointIndices = joint_indices
    db.outputs.positionCommand = position_command
