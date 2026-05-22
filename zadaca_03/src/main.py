import typer
from rich.console import Console
from rich.panel import Panel
from pathlib import Path
import numpy as np

# Core / Domain
from core.config import AppConfig
from core.models import PipelineContext

# Adapters
from adapters.vision_adapter import VisionAdapter
from adapters.robot_adapter import RobotAdapter
from adapters.storage_adapter import StorageAdapter

# Usecases
from usecases.planning_usecase import PlanningUseCase
from usecases.pipeline_orchestrator import PipelineOrchestrator

console = Console()

def setup_components():
    config = AppConfig()
    context = PipelineContext(run_id="run_v2")
    
    # Load camera intrinsics
    try:
        intrinsics = np.load(config.calibration.camera_matrix_path)
    except Exception:
        intrinsics = np.eye(3) # safe fallback for mock
        
    vision = VisionAdapter(config.vision, intrinsics)
    robot = RobotAdapter(config.robot)
    storage = StorageAdapter(config.storage)
    planner = PlanningUseCase(config.trajectory)
    
    orchestrator = PipelineOrchestrator(context, robot, vision, storage, planner)
    return orchestrator


def main(
    offline: bool = typer.Option(False, "--offline", help="Run in offline mode"),
    existing_captures: str = typer.Option(None, "--existing-captures", help="Path to captures directory"),
    mock: bool = typer.Option(False, "--mock", help="Run in mock mode without real robot/camera")
):
    """
    Humanoidna Zadaca 03 - V2 Pipeline
    """
    console.print(Panel.fit("[bold blue]Robot Pick & Place Pipeline V2[/bold blue]"))
    
    orchestrator = setup_components()
    
    if offline and existing_captures:
        console.print(f"[yellow]Running offline mode with captures from: {existing_captures}[/yellow]")
        
        console.print("\n[magenta]=======================================================[/magenta]")
        console.print("[magenta]  KALIBRACIJA: 0: Original | ICP STRATEGIJA: 5: Smart Fitness Rejection  [/magenta]")
        console.print("[magenta]=======================================================[/magenta]")
        
        try:
            orchestrator.run_perception_offline(existing_captures, calib_tweak=0, strategy=5)
            pcd_path = orchestrator.ctx.merged_pcd_path
            if pcd_path:
                import open3d as o3d
                import open3d.visualization.gui as gui
                import open3d.visualization.rendering as rendering
                
                console.print(f"\n[bold green]Prikazujem optimizirani pointcloud (Strategija 5)...[/bold green]")
                console.print("[italic]Zatvorite Open3D prozor za nastavak.[/italic]")
                pcd = o3d.io.read_point_cloud(pcd_path)
                
                # Filtriranje pointclouda (samo stol i voće na njemu)
                # Bounding box oko stola baziran na centroidima
                bbox = o3d.geometry.AxisAlignedBoundingBox(
                    min_bound=(-1.0, -0.4, 0.04), 
                    max_bound=(-0.2,  0.5, 0.25)
                )
                filtered_pcd = pcd.crop(bbox)
                
                # GUI za prikaz teksta i centroida
                app = gui.Application.instance
                app.initialize()
                window = app.create_window("Segmentirano Voće", 1280, 720)
                widget3d = gui.SceneWidget()
                widget3d.scene = rendering.Open3DScene(window.renderer)
                window.add_child(widget3d)
                
                # Dodaj filtriranu scenu
                mat = rendering.MaterialRecord()
                mat.shader = "defaultUnlit"
                mat.point_size = 3.0
                widget3d.scene.add_geometry("scene", filtered_pcd, mat)
                
                # Prikaz centroida, naziva i bounding boxa
                for obj in orchestrator.ctx.segmented_objects:
                    # Bounding Box (Oriented za bolje prianjanje uz voće)
                    bbox_fruit = obj.pcd.get_oriented_bounding_box()
                    bbox_fruit.color = [1, 0, 0] # Ista boja kao i centroid (crvena)
                    
                    b_mat = rendering.MaterialRecord()
                    b_mat.shader = "unlitLine"
                    b_mat.line_width = 3.0
                    widget3d.scene.add_geometry(f"bbox_{obj.instance_id}", bbox_fruit, b_mat)

                    # Mala sfera kao centroid
                    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
                    sphere.paint_uniform_color([1, 0, 0]) # Crvena boja za marker
                    sphere.translate(obj.centroid_robot_base)
                    sphere.compute_vertex_normals()
                    
                    s_mat = rendering.MaterialRecord()
                    s_mat.shader = "defaultLit"
                    widget3d.scene.add_geometry(f"centroid_{obj.instance_id}", sphere, s_mat)
                    
                    # 3D Label: [ID] Klasa
                    label_text = f"[{obj.instance_id}] {obj.class_name}"
                    widget3d.add_3d_label(obj.centroid_robot_base, label_text)
                    
                # Postavi kameru da gleda na stol
                widget3d.setup_camera(60, bbox, [-0.5, 0.15, 0.07])
                app.run()
        except Exception as e:
            console.print(f"[red]Greška: {e}[/red]")
            
        console.print("[bold cyan]Offline obrada uspješno završena. Program se gasi.[/bold cyan]")
        return
        
    # --- Live mode ---
    console.print("[green]Running in LIVE mode[/green]")
    orchestrator.robot.config.use_mock = mock
    
    while True:
        action = typer.prompt("\n[c] Capture new scene, [q] Quit", type=str, default="c")
        if action.lower() == 'q':
            break
        elif action.lower() == 'c':
            orchestrator.run_perception_live()
            
            if not orchestrator.ctx.segmented_objects:
                console.print("[red]No objects found![/red]")
                continue
                
            while True:
                # Prompt for target
                obj_id = typer.prompt("\nEnter object ID to pick, [c] for new capture, [q] to quit", type=str)
                if obj_id.lower() == 'q':
                    return
                if obj_id.lower() == 'c':
                    break
                    
                try:
                    obj_id = int(obj_id)
                except ValueError:
                    console.print("[red]Invalid ID[/red]")
                    continue
                    
                # 2. Planning
                orchestrator.plan_for_object(obj_id)
                
                if orchestrator.ctx.trajectory is None:
                    continue
                    
                # 3. Execution
                orchestrator.execute_current_plan()
                
                console.print("[green]Ready for next pick and place command![/green] (Video can keep rolling)")

if __name__ == "__main__":
    typer.run(main)
