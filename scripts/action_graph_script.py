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
import transforms3d

import numpy as np
import os

from pxr import Usd, UsdGeom, Gf
from omni.isaac.core import World
import omni

world = World.instance()
stage = omni.usd.get_context().get_stage()

# Globals used by helper functions so they don't need `db` in the signature
current_db = None          # will be set at the start of compute()
chain = None               # will be assigned in setup()
updated_position = None    # cached last joint configuration

# absolute path to your URDF file
urdf_path = "/home/milos/Leap-hand-tipcontrol-tutorial/urdf/leap_hand.urdf"

def generate_ikpy_chain(urdf_path):
    """
    Build the IK chain from your URDF file.

    For IKPy to work links and joints must be provided manually in correct order.. 

    base_elements:
        A list describing the ordered sequence of links/joints from
        arm_tool_link → fingertip_end.

    active_links_mask:
        A Boolean list marking which joints are actuated.
        Length must match the number of joints in the chain
        (including IKPy’s dummy root, which is always False).

        In this example (9 joints total):
            Index 0 → IKPy dummy root         (False)
            Index 1 → arm_tool_joint (fixed)  (False)
            Index 2 → dummy_joint (fixed)     (False)
            Index 3 → dummy_joint_2 (fixed)   (False)
            Index 4 → joint_1 (revolute)      (True)
            Index 5 → joint_0 (revolute)      (True)
            Index 6 → joint_2 (revolute)      (True)
            Index 7 → joint_3 (revolute)      (True)
            Index 8 → joint_4 (fixed)         (False)

    Returns:
        An ikpy.Chain object usable for IK and FK.
    """
    return Chain.from_urdf_file(
        urdf_file=urdf_path,
        base_elements=[
            "arm_tool_link",
            "arm_tool_joint",
            "dummy_link",
            "dummy_joint",
            "dummy_link_2",
            "dummy_joint_2",
            "dummy_link_3",
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
        active_links_mask=[False, False, False, False, True, True, True, True, False],
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

def set_translate(prim, oc):
    properties = prim.GetPropertyNames()
    if "xformOp:translate" in properties:
        translate_attr = prim.GetAttribute("xformOp:translate")
        translate_attr.Set(oc)
    elif "xformOp:translation" in properties:
        translation_attr = prim.GetAttribute("xformOp:translation")
        translation_attr.Set(oc)
    elif "xformOp:transform" in properties:
        transform_attr = prim.GetAttribute("xformOp:transform")
        matrix = prim.GetAttribute("xformOp:transform").Get()
        matrix.SetTranslateOnly(oc)
        transform_attr.Set(matrix)
    else:
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(
            UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, ""
        )
        xform_op.Set(oc)

def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def step_euler(current, target, max_step):
    """
    Step one Euler component from current -> target by at most max_step,
    taking the shortest delta in [-pi, pi].
    Returns: (new_value, reached_bool)
    """
    delta = wrap_to_pi(target - current)
    if abs(delta) <= max_step:
        return target, True
    return current + np.sign(delta) * max_step, False


def get_rotate(prim_path):
    """
    Returns current rotation as a 3x3 numpy matrix (best-effort).
    """
    prim = stage.GetPrimAtPath(prim_path)
    properties = prim.GetPropertyNames()

    if "xformOp:orient" in properties:
        quat_gf = prim.GetAttribute("xformOp:orient").Get()  # Gf.Quatd
        real = quat_gf.GetReal()
        imag = quat_gf.GetImaginary()
        q = np.array([real, imag[0], imag[1], imag[2]], dtype=float)  # [w, x, y, z]
        return transforms3d.quaternions.quat2mat(q)

    # Fallback: no orientation set → identity
    return np.eye(3, dtype=float)

def set_rotate(prim, rot_mat):
    """
    Store orientation as a quaternion in xformOp:orient.
    rot_mat: 3x3 numpy-like rotation matrix.
    """
    # Ensure 3x3 numpy array
    rot_np = np.array(rot_mat, dtype=float).reshape(3, 3)

    # 3x3 -> quaternion [w, x, y, z]
    q = transforms3d.quaternions.mat2quat(rot_np)
    quat_gf = Gf.Quatd(q[0], q[1], q[2], q[3])

    properties = prim.GetPropertyNames()

    if "xformOp:orient" in properties:
        # Reuse existing orient op
        orient_attr = prim.GetAttribute("xformOp:orient")
        orient_attr.Set(quat_gf)
    else:
        # Create new orient op
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(
            UsdGeom.XformOp.TypeOrient,
            UsdGeom.XformOp.PrecisionDouble,
            ""
        )
        xform_op.Set(quat_gf)


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

def move_arm_tool_xyzrpy(x=0, y=0, z=0.02, roll=0, pitch=0, yaw=0):
    """
    Incrementally moves `/arm_tool_link` toward the specified pose in XYZ and RPY.

    Position is updated along a straight-line path, while orientation is
    interpolated in Euler space (sxyz) using shortest-angle steps.
    """

    arm_tool_link_path = "/World/leap_hand/arm_tool_link"
    arm_tool_link_prim = stage.GetPrimAtPath(arm_tool_link_path)

    # --------------- POSITION STEP ---------------
    target_pos = np.array([x, y, z], dtype=float)
    cur_t = get_translation(arm_tool_link_path)
    cur_pos = np.array([cur_t[0], cur_t[1], cur_t[2]], dtype=float)

    delta = target_pos - cur_pos
    dist = np.linalg.norm(delta)
    max_pos_step = 0.005  # 5 mm per compute

    if dist < 1e-5:
        new_pos = target_pos
        pos_reached = True
    else:
        step = min(max_pos_step, dist)
        new_pos = cur_pos + (delta / dist) * step
        pos_reached = dist <= max_pos_step

    set_translate(arm_tool_link_prim, Gf.Vec3d(new_pos[0], new_pos[1], new_pos[2]))

    # Current rotation from orient (3x3)
    cur_rot = get_rotate(arm_tool_link_path)
    cur_roll, cur_pitch, cur_yaw = transforms3d.euler.mat2euler(
        cur_rot, axes="sxyz"
    )

    target_roll = roll
    target_pitch = pitch
    target_yaw = yaw

    max_ang_step = 0.05  # rad per compute

    new_roll, r_done = step_euler(cur_roll,  target_roll,  max_ang_step)
    new_pitch, p_done = step_euler(cur_pitch, target_pitch, max_ang_step)
    new_yaw, y_done = step_euler(cur_yaw,   target_yaw,   max_ang_step)

    new_rot = transforms3d.euler.euler2mat(new_roll, new_pitch, new_yaw, axes="sxyz")
    set_rotate(arm_tool_link_prim, new_rot)

    rot_reached = r_done and p_done and y_done

    return pos_reached and rot_reached


def move_fingertip_tool_xyzrpy(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    """
        high-level API to calculate the joint_angles by specifying a target XYZ/RPY
    """
    global chain, updated_position

    # if chain is None:
    #     print("move_fingertip_tool_xyzrpy: IK chain not initialized yet.")
    #     return None

    target = np.array([x, y, z], dtype=float)

    # If we don't have a previous solution, start from zeros
    if updated_position is None:
        # 9 DoFs in your original code (ikpy dummy + arm_tool_joint + urdf_dummy_joint + urdf_dummy_joint_2 + 4 actuated + end-effector)
        initial_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        initial_position = updated_position
    
    # Solve IK (orientation parameters can be added later if needed)
    joint_angles = chain.inverse_kinematics(
        target_position=target,
        # target_orientation=...
        # orientation_mode="X",
        initial_position=initial_position,
    )

    # update the global cached position for the next call
    updated_position = joint_angles.tolist()

    return joint_angles

def move_fingertip_tool_delta_ee(dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0):
    """
    Move fingertip *relative* to the current fingertip_end pose (EE frame).

    (dx, dy, dz, droll, dpitch, dyaw) are offsets expressed in the fingertip_end frame.
    For now we only use translation; rotation deltas are kept for future extension.
    """
    global chain, updated_position

    # Current joint configuration: last IK solution or neutral
    if updated_position is None:
        # 9 DoFs: [ikpy_dummy, arm_tool_joint, dummy_joint, dummy_joint_2,
        #          joint_1, joint_0, joint_2, joint_3, joint_4]
        current_position = np.zeros(9, dtype=float)
    else:
        current_position = np.array(updated_position, dtype=float)

    # FK: fingertip_end pose in arm_tool_link (base) frame
    T_current = chain.forward_kinematics(current_position, full_kinematics=False)

    # --- Build local EE-frame offset transform -------------------
    T_delta = np.eye(4, dtype=float)

    # 1) Translation in EE frame
    T_delta[:3, 3] = np.array([dx, dy, dz], dtype=float)

    # 2) Rotation in EE frame (currently NOT applied)
    # rot_delta = transforms3d.euler.euler2mat(droll, dpitch, dyaw, axes="sxyz")
    # T_delta[:3, :3] = rot_delta

    # -------------------------------------------------------------
    # Global target pose in base frame
    T_target = T_current @ T_delta
    target_pos = T_target[:3, 3]
    # target_rot = T_target[:3, :3]

    # Run IK to reach the new global target
    joint_angles = chain.inverse_kinematics(
        target_position=target_pos,
        # target_orientation=target_rot,
        # orientation_mode="X",
        initial_position=current_position,
    )

    # Cache for next calls
    updated_position = joint_angles.tolist()

    # Return both the solution and the global target position we just computed
    return joint_angles, target_pos


def move_arm_and_fingertip_tool_xyzrpy(arm_x= 0, arm_y= 0, arm_z=0, arm_roll=0, arm_pitch=0, arm_yaw=0, hand_x= 0, hand_y= 0, hand_z=0, hand_roll=0, hand_pitch=0, hand_yaw=0):
    """
        Moves both arm_tool_link and fingertip_tool_link in XYZ + RPY.
    """
    arm_tool_reached = move_arm_tool_xyzrpy(x=arm_x, y=arm_y, z=arm_z, roll=arm_roll, pitch=arm_pitch, yaw=arm_yaw)

    hand_joint_angles = move_fingertip_tool_xyzrpy(x=hand_x, y=hand_y, z=hand_z, roll=hand_roll, pitch=hand_pitch, yaw=hand_yaw)

    return hand_joint_angles, arm_tool_reached

# helper to check if the arm reached the target
def check_fingertip_reachable(target, joint_angles, tolerance=2e-3):
    """
    Checks if the fingertip reached `target` within `tolerance` meters.

    Returns:
        reached (bool), error (float), ee_position (np.ndarray or None)
    """
    global chain

    if chain is None or joint_angles is None:
        return False, None, None

    T_fk = chain.forward_kinematics(joint_angles, full_kinematics=False)
    ee_position = T_fk[:3, 3]
    error = np.linalg.norm(ee_position - target)
    reached = error <= tolerance
    return reached, error, ee_position

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
    # Initialize state machine variables
    db.per_instance_state.phase = 0
    db.per_instance_state.phase_frames = 0

    # sync globals with per-instance state for use in helper functions
    global chain, updated_position
    chain = db.per_instance_state.chain
    updated_position = db.per_instance_state.updated_position

def cleanup(db: og.Database):
    db.outputs.robotPath = ""
    db.outputs.jointIndices = []
    db.outputs.positionCommand = []

    global chain, updated_position, current_db
    chain = None
    updated_position = None
    current_db = None


def compute(db: og.Database):
    # remember the current db so helpers could use it if needed
    global current_db, chain, updated_position
    current_db = db

    # Ensure shared variables are initialized
    if not db.per_instance_state.initialized:
        cleanup(db)
        setup(db)

    # Keep globals in sync with per-instance state
    chain = db.per_instance_state.chain
    updated_position = db.per_instance_state.updated_position

    # Get the robot path and joints map
    robot_path = db.per_instance_state.robot_path
    joints_map = db.per_instance_state.joints_map

    db.log_info("==== JOINTS_MAP ====")
    db.log_info(str(joints_map))

    # ---------------------------
    # Phase machine
    # ---------------------------
    # phase 0: move_arm_tool_xyzrpy (arm base pose)
    # phase 1: fingertip → custom target 0 (absolute, arm_tool_link frame)
    # phase 2: fingertip → custom target 1 (absolute, arm_tool_link frame)
    # phase 3: fingertip → EE-frame delta motion (relative move in fingertip_end frame)
    # phase 4: move_arm_and_fingertip_tool_xyzrpy
    # phase 5: fingertip → target_frame from sim

    phase = db.per_instance_state.phase
    phase_frames = db.per_instance_state.phase_frames

    HOLD_FRAMES = 60

    # 2 custom fingertip targets in arm_tool_link frame (absolute)
    custom_targets = [
        np.array([0.16, 0.05, -0.03], dtype=float),
        np.array([0.17, 0.07, -0.04], dtype=float),
    ]

    # EE-frame delta for phase 3 (e.g., slide 2 mm along local -X of fingertip_end)
    delta_ee = np.array([-0.002, 0.00, 0.00], dtype=float)

    # Common variables for IK logging
    joint_angles = None
    target_vec = None
    reachable = False
    error = 0.0
    ee_position = np.array([0.0, 0.0, 0.0], dtype=float)
    did_compute_ik = False

    # ============================================================
    # PHASE 0: move_arm_tool_xyzrpy
    # ============================================================
    if phase == 0:
        arm_tool_reached = move_arm_tool_xyzrpy(
            x=0.0,
            y=0.0,
            z=0.2,
            roll=3.14,
            pitch=0.0,
            yaw=0.0,
        )

        if arm_tool_reached:
            db.log_warning("✅ Phase 0: arm_tool_link base pose reached. Switching to phase 1.")
            phase = 1
            phase_frames = 0
        else:
            db.log_warning("⏳ Phase 0: arm_tool_link moving towards base pose...")

    # ============================================================
    # PHASE 1–2: fingertip moves through 2 custom absolute targets
    # ============================================================
    elif phase in (1, 2):
        idx = phase - 1  # 0, 1
        target_vec = custom_targets[idx]
        target_x, target_y, target_z = target_vec

        joint_angles = move_fingertip_tool_xyzrpy(
            x=target_x,
            y=target_y,
            z=target_z,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )
        reachable, error, ee_position = check_fingertip_reachable(target_vec, joint_angles)
        did_compute_ik = True

        db.log_warning(
            f"Phase {phase}: moving fingertip to custom target {idx} "
            f"[{target_x:.3f}, {target_y:.3f}, {target_z:.3f}]"
        )

        phase_frames += 1
        if phase_frames >= HOLD_FRAMES:
            phase += 1
            phase_frames = 0
            if phase == 2:
                db.log_warning("Switching to phase 2 (second fingertip target).")
            else:
                db.log_warning("Switching to phase 3 (EE-frame delta motion).")

    # ============================================================
    # PHASE 3: fingertip EE-frame delta motion
    # ============================================================
    elif phase == 3:
        dx, dy, dz = delta_ee

        # Only compute the delta-based IK once when entering phase 3
        if phase_frames == 0:
            joint_angles, target_vec = move_fingertip_tool_delta_ee(
                dx=dx,
                dy=dy,
                dz=dz,
                droll=0.0,
                dpitch=0.0,
                dyaw=0.0,
            )
            # Cache for later frames in this phase
            db.per_instance_state.phase3_joint_angles = joint_angles.tolist()
            db.per_instance_state.phase3_target_vec = target_vec.tolist()

            target_x, target_y, target_z = target_vec
            db.log_warning(
                "Phase 3: moving fingertip by EE delta "
                f"[dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}] "
                f"→ global target [{target_x:.3f}, {target_y:.3f}, {target_z:.3f}]"
            )
        else:
            joint_angles = np.array(db.per_instance_state.phase3_joint_angles, dtype=float)
            target_vec = np.array(db.per_instance_state.phase3_target_vec, dtype=float)

        reachable, error, ee_position = check_fingertip_reachable(target_vec, joint_angles)
        did_compute_ik = True

        phase_frames += 1
        if phase_frames >= HOLD_FRAMES:
            phase = 4
            phase_frames = 0
            db.log_warning("Switching to phase 4 (arm+fingertip combined move).")

    # ============================================================
    # PHASE 4: move with move_arm_and_fingertip_tool_xyzrpy
    # ============================================================
    elif phase == 4:
        arm_target = dict(x=0.1, y=0.0, z=0.25, roll=3.14, pitch=0.2, yaw=0.0)
        hand_target = dict(x=0.06, y=0.13, z=-0.035, roll=0.0, pitch=0.0, yaw=0.0)

        hand_joint_angles, arm_tool_reached = move_arm_and_fingertip_tool_xyzrpy(
            arm_x=arm_target["x"],
            arm_y=arm_target["y"],
            arm_z=arm_target["z"],
            arm_roll=arm_target["roll"],
            arm_pitch=arm_target["pitch"],
            arm_yaw=arm_target["yaw"],
            hand_x=hand_target["x"],
            hand_y=hand_target["y"],
            hand_z=hand_target["z"],
            hand_roll=hand_target["roll"],
            hand_pitch=hand_target["pitch"],
            hand_yaw=hand_target["yaw"],
        )

        target_vec = np.array(
            [hand_target["x"], hand_target["y"], hand_target["z"]], dtype=float
        )

        reachable, error, ee_position = check_fingertip_reachable(target_vec, hand_joint_angles)
        did_compute_ik = True

        db.log_warning("Phase 4: final arm+fingertip pose set.")

        if arm_tool_reached:
            db.log_warning("✅ Phase 4: arm base pose + fingertip target reached. Switching to phase 5.")
            phase = 5
            phase_frames = 0
        else:
            db.log_warning("⏳ Phase 4: moving arm towards final pose, fingertip tracking target...")

    # ============================================================
    # PHASE 5: fingertip follows target_frame from sim
    # ============================================================
    elif phase == 5:
        target_from_frame = get_translation(
            "/World/leap_hand/arm_tool_link/target_frame"
        )
        target_x, target_y, target_z = target_from_frame
        target_vec = np.array([target_x, target_y, target_z], dtype=float)

        joint_angles = move_fingertip_tool_xyzrpy(
            x=target_x,
            y=target_y,
            z=target_z,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )
        reachable, error, ee_position = check_fingertip_reachable(target_vec, joint_angles)
        did_compute_ik = True

        db.log_warning(
            "Phase 5: moving fingertip to target_frame from sim "
            f"[{target_x:.3f}, {target_y:.3f}, {target_z:.3f}]"
        )
        # We stay in phase 5, continuously following that frame.

    # ============================================================
    # Common outputs and logging
    # ============================================================
    # If we did IK this frame, update stored solution and log it.
    if did_compute_ik and joint_angles is not None:
        db.per_instance_state.updated_position = joint_angles.tolist()
        updated_position = db.per_instance_state.updated_position

        target_str = "[{:.4f}, {:.4f}, {:.4f}]".format(
            target_vec[0], target_vec[1], target_vec[2]
        )

        if reachable:
            db.log_warning(
                "✅ Target reachable "
                f"(phase={phase}, target={target_str}, error={error:.6f} m)"
            )
        else:
            db.log_warning(
                "❌ Target unreachable "
                f"(phase={phase}, target={target_str}, error={error:.6f} m)"
            )

        db.log_warning(
            "IK solution:\n"
            "  Calculated finger joints (rad):\n"
            "    1: {j1:.4f}, 0: {j0:.4f}, 2: {j2:.4f}, 3: {j3:.4f}\n"
            "  EE position:   [{ee_x:.6f}, {ee_y:.6f}, {ee_z:.6f}]\n"
            "  Target:        [{tx:.6f}, {ty:.6f}, {tz:.6f}]\n"
            "  Position error: {err:.8f} m".format(
                j1=joint_angles[4],  # joint_1
                j0=joint_angles[5],  # joint_0
                j2=joint_angles[6],  # joint_2
                j3=joint_angles[7],  # joint_3
                ee_x=ee_position[0],
                ee_y=ee_position[1],
                ee_z=ee_position[2],
                tx=target_vec[0],
                ty=target_vec[1],
                tz=target_vec[2],
                err=error,
            )
        )
        position_command = joint_angles[4:8].tolist()

    else:
        # No IK this frame (phase 0): reuse last solution if we have one, else zeros.
        if updated_position is not None:
            position_command = updated_position[4:8]
        else:
            position_command = [0.0, 0.0, 0.0, 0.0]

    # Save phase state
    db.per_instance_state.phase = phase
    db.per_instance_state.phase_frames = phase_frames

    joint_names = list(joints_map.keys())
    joint_indices = list(joints_map.values())

    db.outputs.robotPath = robot_path
    db.outputs.jointIndices = joint_indices
    db.outputs.positionCommand = position_command
