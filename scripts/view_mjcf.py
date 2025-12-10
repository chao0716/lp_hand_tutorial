#!/usr/bin/env python3
import sys
import time
import mujoco
import mujoco.viewer

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} path_to_model.xml")
        sys.exit(1)

    model_path = sys.argv[1]
    print(f"Loading model from: {model_path}")

    # Load model and create data
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Open passive viewer ? no controls, just integration
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            step_start = time.time()

            # advance simulation (no ctrl set)
            mujoco.mj_step(model, data)
            viewer.sync()

            # simple real-time pacing
            dt = model.opt.timestep
            remaining = dt - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)

if __name__ == "__main__":
    main()
