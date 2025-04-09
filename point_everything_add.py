from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Simulation started")

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
from omni.isaac.debug_draw import _debug_draw
import omni.usd
import carb
import omni

# Create the world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Acquire the debug draw interface
draw = _debug_draw.acquire_debug_draw_interface()

def get_scene_object_positions():
    """
    Get positions of all objects in the scene
    """
    # Get the current stage
    stage = omni.usd.get_context().get_stage()
    
    # List to store object positions
    object_positions = []
    
    # Iterate through all prims in the stage
    for prim in stage.Traverse():
        # Check if prim is a geometry (renderable object)
        if prim.IsValid() and prim.GetTypeName() in ['Cube', 'Sphere', 'Cylinder', 'Mesh', 'Xform']:
            try:
                # Try to get world transform
                xform = omni.usd.get_world_transform_matrix(prim)
                
                # Extract position from transform matrix
                position = xform.ExtractTranslation()
                
                object_positions.append({
                    'path': prim.GetPath(),
                    'position': [position[0], position[1], position[2]]
                })
            except Exception as e:
                print(f"Could not get position for {prim.GetPath()}: {e}")
    
    return object_positions

def update_lines():
    # Get current object positions
    object_positions = get_scene_object_positions()
    
    # Prepare lists for line drawing
    line_starts = []
    line_ends = []
    colors = []
    sizes = []
    
    # Define fixed start point
    start_point = [10, 0, 10]
    
    # Generate lines to each object
    color_options = [
        [1, 0, 0, 1],    # Red
        [0, 1, 0, 1],    # Green
        [0, 0, 1, 1],    # Blue
        [1, 1, 0, 1],    # Yellow
        [1, 0, 1, 1],    # Magenta
        [0, 1, 1, 1]     # Cyan
    ]
    
    for i, obj in enumerate(object_positions):
        line_starts.append(start_point)
        line_ends.append(obj['position'])
        
        # Cycle through colors
        colors.append(color_options[i % len(color_options)])
        sizes.append(5)  # Line thickness
    
    # Clear previous lines
    draw.clear_lines()
    
    # Draw the lines if we have any objects
    if line_starts:
        draw.draw_lines(line_starts, line_ends, colors, sizes)

# Add some example objects to the scene
# Cubes
world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube1",
        name="cube1",
        position=np.array([0, 0, 0.6]),
        scale=np.array([1, 1, 1]),
        color=np.array([0.7, 0.5, 1.0])
    )
)

world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube2",
        name="cube2",
        position=np.array([2, 0, 0.6]),
        scale=np.array([1, 1, 1]),
        color=np.array([1.0, 0.5, 0.5])
    )
)

# Simulation loop
try:
    frame_count = 0
    while simulation_app.is_running():
        # Step the simulation
        world.step(render=True)
        
        # Update lines every frame
        update_lines()
        
        # Print object positions every 200 frames
        if frame_count % 200 == 0:
            objects = get_scene_object_positions()
            print(f"Frame {frame_count}: Objects in scene:")
            for obj in objects:
                print(f"  {obj['path']}: {obj['position']}")
        
        frame_count += 1

except KeyboardInterrupt:
    print("Simulation stopped by user")

finally:
    # Properly close the simulation app
    simulation_app.close()