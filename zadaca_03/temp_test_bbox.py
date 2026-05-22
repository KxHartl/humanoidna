import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

def test():
    app = gui.Application.instance
    app.initialize()

    window = gui.Application.instance.create_window("Test", 800, 600)
    widget3d = gui.SceneWidget()
    widget3d.scene = rendering.Open3DScene(window.renderer)
    window.add_child(widget3d)

    # create a sphere
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    bbox = mesh.get_axis_aligned_bounding_box()
    bbox.color = [0, 1, 0]
    
    mat = rendering.MaterialRecord()
    mat.shader = "unlitLine"
    mat.line_width = 3.0
    
    # Can we add bbox directly?
    try:
        widget3d.scene.add_geometry("bbox", bbox, mat)
        print("Success adding bbox directly!")
    except Exception as e:
        print(f"Failed to add bbox directly: {e}")
        try:
            ls = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(bbox)
            ls.paint_uniform_color([0, 1, 0])
            widget3d.scene.add_geometry("bbox_ls", ls, mat)
            print("Success adding LineSet!")
        except Exception as e2:
            print(f"Failed LineSet: {e2}")

if __name__ == "__main__":
    test()
