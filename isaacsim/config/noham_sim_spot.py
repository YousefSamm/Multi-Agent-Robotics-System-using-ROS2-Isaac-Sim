import time
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.prims import get_prim_at_path
from spot import SpotFlatTerrainPolicy


def setup_simulation():
    """Initialize simulation environment and enable OmniGraphs with robust initialization"""
    print("Setting up simulation environment...")
    
    # Load stage
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stage_path = os.path.join(
        script_dir, 
        "Collected_noham_sim_spot", 
        "noham_sim_spot.usd"
    )
    print(f"Loading stage: {stage_path}")
    open_stage(stage_path)
    
    # Enable essential extensions
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    required_extensions = [
        "omni.graph.core", 
        "omni.graph.action", 
        "isaacsim.ros2.bridge"
    ]
    
    print("Enabling required extensions...")
    for ext in required_extensions:
        if not ext_manager.is_extension_enabled(ext):
            print(f"  Enabling {ext}...")
            ext_manager.set_extension_enabled_immediate(ext, True)
    
    # Wait for extensions to fully load with status checking
    print("Waiting for extensions to fully load...")
    max_wait_time = 15.0  # Increased timeout to 15 seconds
    start_time = time.time()
    
    while time.time() - start_time < max_wait_time:
        all_loaded = True
        for ext in required_extensions:
            if not ext_manager.is_extension_enabled(ext):
                all_loaded = False
                break
        if all_loaded:
            print("  All extensions loaded successfully!")
            break
        simulation_app.update()
        time.sleep(0.1)
    
    if not all_loaded:
        print("Warning: Some extensions failed to load within timeout")
    
    # Wait for USD stage to be fully loaded
    print("Waiting for USD stage to fully load...")
    max_wait_time = 10.0
    start_time = time.time()
    
    while time.time() - start_time < max_wait_time:
        stage = omni.usd.get_context().get_stage()
        if stage and stage.GetRootLayer():
            print("  USD stage loaded successfully!")
            break
        simulation_app.update()
        time.sleep(0.1)
    
    # Start timeline FIRST (before enabling graphs)
    print("Starting timeline...")
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    
    # Wait a moment for timeline to stabilize
    print("Waiting for timeline to stabilize...")
    for i in range(20):  # Increased wait time
        simulation_app.update()
        time.sleep(0.1)
    
    # Wait for OmniGraphs to be fully loaded and available
    print("Waiting for OmniGraphs to load...")
    max_wait_time = 10.0
    start_time = time.time()
    
    while time.time() - start_time < max_wait_time:
        stage = omni.usd.get_context().get_stage()
        if stage:
            graphs_found = False
            for prim in stage.Traverse():
                if "Graph" in str(prim.GetPath()):
                    try:
                        graph = og.get_graph_by_path(str(prim.GetPath()))
                        if graph and hasattr(graph, 'is_valid'):
                            graphs_found = True
                            break
                    except Exception:
                        continue
            if graphs_found:
                print("  OmniGraphs are available!")
                break
        simulation_app.update()
        time.sleep(0.2)
    
    # NOW enable all graphs in stage (after timeline is running and graphs are loaded)
    print("Enabling OmniGraphs...")
    stage = omni.usd.get_context().get_stage()
    if stage:
        for prim in stage.Traverse():
            if "Graph" in str(prim.GetPath()):
                try:
                    graph = og.get_graph_by_path(str(prim.GetPath()))
                    if (graph and hasattr(graph, 'is_valid') and 
                        graph.is_valid()):
                        if (hasattr(graph, 'is_disabled') and 
                            graph.is_disabled()):
                            print(f"  Enabling graph: {prim.GetPath()}")
                            graph.set_disabled(False)
                    elif (graph and hasattr(graph, 'is_disabled') and 
                          graph.is_disabled()):
                        # Try to enable even if is_valid() fails
                        print(f"  Attempting to enable graph: "
                              f"{prim.GetPath()}")
                        try:
                            graph.set_disabled(False)
                        except Exception:
                            pass
                except Exception as e:
                    print(f"  Warning: Could not enable graph "
                          f"{prim.GetPath()}: {e}")
                    continue
    
    # Wait for graphs to be fully initialized and running
    max_wait_time = 10.0  # Increased timeout to 10 seconds
    start_time = time.time()
    
    while time.time() - start_time < max_wait_time:
        all_graphs_ready = True
        stage = omni.usd.get_context().get_stage()
        if stage:
            for prim in stage.Traverse():
                if "Graph" in str(prim.GetPath()):
                    try:
                        graph = og.get_graph_by_path(str(prim.GetPath()))
                        if (graph and hasattr(graph, 'is_valid') and 
                            graph.is_valid()):
                            if (hasattr(graph, 'is_running') and 
                                not graph.is_running()):
                                all_graphs_ready = False
                                break
                    except Exception:
                        # Don't print warnings here to avoid spam
                        continue
        
        if all_graphs_ready:
            print("  All graphs are running!")
            break
        simulation_app.update()
        time.sleep(0.2)
    
    if not all_graphs_ready:
        print("Warning: Some graphs may not be fully initialized")
    
    # Final initialization updates
    print("Final initialization updates...")
    for i in range(50):
        simulation_app.update()
        if i % 10 == 0:
            print(f"  Update {i+1}/50")
    
    print("Simulation setup completed!")


def main():
    # Setup simulation environment
    setup_simulation()
    
    # Initialize world and robot
    world = World(
        stage_units_in_meters=1.0, 
        physics_dt=1/500, 
        rendering_dt=1/50  # rendering_dt should be 1/50
    )
    robot = SpotFlatTerrainPolicy(
        prim_path="/World/spot", 
        name="Spot", 
        position=np.array([0, 0, 0.8])
    )
    
    world.reset()
    robot.initialize()
    
    # Get world prim for velocity commands
    world_prim = get_prim_at_path("/World")
    base_command = np.zeros(3)
    
    # Physics callback
    def physics_step(step_size):
        robot.forward(step_size, base_command)
    
    world.add_physics_callback("physics_step", callback_fn=physics_step)
    
    # Main simulation loop
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing():
            try:
                ang_vel = world_prim.GetAttribute("angular_vel").Get()
                lin_vel = world_prim.GetAttribute("linear_vel").Get()
                if ang_vel and lin_vel:
                    base_command[:] = [lin_vel[0], lin_vel[1], ang_vel[2]]
            except Exception:
                pass
    
    simulation_app.close()


if __name__ == "__main__":
    main()
