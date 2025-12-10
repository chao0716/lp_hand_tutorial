# LeapHand Tip Control, Frame & IK Tutorial  
*A practical guide for defining frames, understanding transformations, and generating targets for arm, finger, or combined kinematic chains.*

---

## Understanding Frames in URDF

A URDF is essentially a **tree of coordinate systems**:

- Every **link** defines a *frame*.
- Every **joint** defines how one frame moves relative to another (revolute, prismatic, fixed, etc.).

Typical robotic arm example:

```text
arm_base_link → arm_link_1 → arm_link_2 → arm_end_effector_link
```

The *first link* in a chain is called the **base_link** (or `arm_base_link`, `finger_base_link`, `dummy_link`, etc.).  
All target positions you send to IK are **expressed relative to this base link** unless you explicitly transform them.

In practice for arms/fingers you might have, for example:

```text
arm_base_link   → arm_link_1   → arm_link_2   → arm_end_effector_link
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
    active_links_mask=[False, True, True, False],  # [ikpy_root, arm_joint_0, arm_joint_1, arm_joint_2]
)
```

**Notes about `active_links_mask`:**

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
    active_links_mask=[False, True, True, False],  # [ikpy_root, finger_joint_0, finger_joint_1, finger_joint_2]
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
        False,  # ikpy_root (internal)
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

### Transform of *each* link in the chain

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

## Example usage of ikpy for leap_hand.urdf robot description

The script in which the ikpy implementation is used is action_graph_script.py. Bellow is a short explanation of the code and what part is where used.

### State Machine (Phase Machine) Overview

The Isaac Sim script uses a small **phase machine** inside `compute()` to
demonstrate different types of motion and target definitions over time.

At a high level:

- **Phase 0 – Arm setup**  
  The arm tool link (`arm_tool_link`) is moved to a defined “base pose” using `move_arm_tool_xyzrpy(...)`.  
  This gives a consistent starting configuration.

- **Phases 1–2 – Absolute fingertip targets**  
  The fingertip is moved to a small list of **absolute 3D targets**, expressed in the base frame, using `move_fingertip_tool_xyzrpy(...)` and IK.  
  This shows classic “end-effector to target point” behavior.

- **Phase 3 – EE-frame delta motion**  
  Instead of a fixed world target, the fingertip moves by a small **offset in its own local frame** using something like `move_fingertip_tool_delta_ee(dx, dy, dz, ...)`.  
  Internally, FK gives the current fingertip pose, a local offset transform is applied,
  and that result is sent back to IK as a new global target.  
  This illustrates the pattern: *“global target = current pose × local delta”*.

- **Phase 4 – Combined arm + fingertip move**  
  `move_arm_and_fingertip_tool_xyzrpy(...)` adjusts both arm pose and fingertip target
  in a coordinated way, showing how an **arm+hand chain** can be controlled together.

- **Phase 5 – Following a scene frame**  
  The fingertip tracks a `/World/.../target_frame` prim in the USD stage.  
  Each frame, the script reads that prim’s translation, converts it into a target position,
  runs IK, and commands the joints. This demonstrates **driving IK from a frame in the scene**.

Each phase reuses the same ideas from above (FK, chain selection, targets in different frames), but in a controlled sequence that you can observe step-by-step in the simulator.

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
- The phase machine ties this all together by running through a sequence of
  examples: arm setup, absolute fingertip targets, EE-relative deltas, combined arm+finger motion, and following a target frame in the scene.

With this, you can:

- Drive only the finger, only the arm, or both.
- Express targets in **any frame** in the chain, then convert them back to the appropriate base_link.
- Keep a clean mental model of your frames and transformations, which is crucial for stable tip control.
