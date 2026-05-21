import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(
    r"C:\mujoco\mujoco_menagerie\unitree_h1\scene.xml"
)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        data.ctrl[:] = 0.0  # nema upravljanja
        mujoco.mj_step(model, data)
        viewer.sync()
