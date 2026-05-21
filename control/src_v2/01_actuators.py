import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path(
    r"C:\mujoco\mujoco_menagerie\unitree_h1\scene.xml"
)

print(f"Broj aktuatora (nu): {model.nu}\n")

for i in range(model.nu):

    # ime aktuatora
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        i
    )

    # kojem jointu pripada
    trnid = model.actuator_trnid[i][0]

    joint_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        trnid
    )

    # DOF index jointa
    dof_id = model.jnt_dofadr[trnid]

    # qpos index
    qpos_id = model.jnt_qposadr[trnid]

    # control range
    ctrl_range = model.actuator_ctrlrange[i]

    print(f"ACTUATOR {i}")
    print(f"  actuator name : {name}")
    print(f"  joint name    : {joint_name}")
    print(f"  dof index     : {dof_id}")
    print(f"  qpos index    : {qpos_id}")
    print(f"  ctrl range    : {ctrl_range}")
    print()
