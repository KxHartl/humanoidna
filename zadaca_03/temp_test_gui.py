import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np

def test():
    app = gui.Application.instance
    app.initialize()

    window = gui.Application.instance.create_window("Test", 800, 600)
    widget3d = gui.SceneWidget()
    widget3d.scene = rendering.Open3DScene(window.renderer)
    window.add_child(widget3d)

    # create a sphere
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    mesh.compute_vertex_normals()
    mat = rendering.MaterialRecord()
    mat.shader = "defaultLit"
    widget3d.scene.add_geometry("sphere", mesh, mat)
    
    widget3d.add_3d_label([0, 0, 0.15], "Test Label")
    
    # We will just run for 1 frame and close it to verify it works headless-ish or doesn't crash
    # Actually wait, we just want to see if the import and attributes exist
    print("GUI classes exist and are accessible!")
    
if __name__ == "__main__":
    test()
