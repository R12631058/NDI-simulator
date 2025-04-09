from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Simulation started")

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
from omni.isaac.debug_draw import _debug_draw
import omni.usd
import carb

# Create the world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Add multiple dynamic cuboids
cubes = []
cube_positions = [
    np.array([0, 0, 0.6]),
    np.array([2, 0, 0.6]),
    np.array([0, 2, 0.6]),
    np.array([2, 2, 0.6])
]

cube_colors = [
    np.array([0.7, 0.5, 1.0]),  # Purple
    np.array([1.0, 0.5, 0.5]),  # Salmon
    np.array([0.5, 1.0, 0.5]),  # Light Green
    np.array([0.5, 0.5, 1.0])   # Light Blue
]

for i in range(4):
    cube = world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/my_cube_{i}",
            name=f"my_cube_{i}",
            position=cube_positions[i],
            scale=np.array([1, 1, 1]),
            color=cube_colors[i]
        )
    )
    cubes.append(cube)

# Acquire the debug draw interface
draw = _debug_draw.acquire_debug_draw_interface()

def get_cube_positions():
    """
    Get the positions of all cubes in the World
    """
    return [cube.get_world_pose()[0].tolist() for cube in cubes]

def update_lines():
    # Get current cube positions
    cube_positions = get_cube_positions()
    
    # Define fixed start points (all starting from (10, 0, 10))
    line_starts = [[10, 0, 10]] * len(cubes)
    
    # Line ends are the current cube positions
    line_ends = cube_positions
    
    # Define colors for each line (one for each cube)
    colors = [
        [1, 0, 0, 1],    # Red
        [0, 1, 0, 1],    # Green
        [0, 0, 1, 1],    # Blue
        [1, 1, 0, 1]     # Yellow
    ]
    
    # Define line thicknesses
    sizes = [5] * len(cubes)
    
    # Clear previous lines before drawing new ones
    draw.clear_lines()
    
    # Draw the lines
    draw.draw_lines(line_starts, line_ends, colors, sizes)

# Simulation loop
try:
    frame_count = 0
    while simulation_app.is_running():
        # Step the simulation
        world.step(render=True)
        
        # Update lines every frame to always show current positions
        update_lines()
        
        # Print positions of all cubes every 200 frames
        if frame_count % 200 == 0:
            for i, cube in enumerate(cubes):
                position, _ = cube.get_world_pose()
                print(f"Frame {frame_count}: Cube {i} Position: {position}")
        
        frame_count += 1

except KeyboardInterrupt:
    print("Simulation stopped by user")

finally:
    # Properly close the simulation app
    simulation_app.close()