from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Simulation started")

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.debug_draw import _debug_draw
import numpy as np
import omni.usd
import carb
import omni
import omni.kit.raycast.query
from pxr import Gf, Usd, UsdGeom

# Create the world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Specific path to the USD file
USD_FILE_PATH = "file://C:/Nick/surgery_team/surgery_team/BM.usd"

# Acquire the debug draw interface, raycast interface for visualization
draw = _debug_draw.acquire_debug_draw_interface()
raycast = omni.kit.raycast.query.acquire_raycast_query_interface()
line_starts = []
line_ends = []
raycast_results = []

def import_usd_model(file_path, position):
    """
    Import USD file as a model at specified position
    """
    try:
        # Add reference to the stage at the specified position
        prim_path = "/World/Marker"
        add_reference_to_stage(file_path, prim_path)
        
        # Get the stage
        stage = omni.usd.get_context().get_stage()
        
        # Get the prim we just added
        prim = stage.GetPrimAtPath(prim_path)
        
        if prim.IsValid():
            # Set the translation (position)
            xform_op = prim.GetAttribute('xformOp:translate')
            if xform_op:
                xform_op.Set(position)
            
            print(f"USD model imported successfully at {position}")
            return prim
        else:
            print("Failed to add USD model to stage")
            return None
    except Exception as e:
        print(f"Error importing USD model: {e}")
        return None

#Unused function
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
        if prim.IsValid() and prim.GetTypeName() in ['Xform']:
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


def get_marker_rball_positions(marker_name="Marker", rball_prefix="rball_"):
    """
    Get positions of rball objects under the specified marker in the scene.

    :param marker_name: The name of the marker group to look for.
    :param rball_prefix: The prefix of the rball objects to filter.
    :return: A list of positions of the rball objects.
    """
    # Get the current stage
    stage = omni.usd.get_context().get_stage()

    # List to store rball positions
    rball_positions = []

    # Iterate through all prims in the stage
    for prim in stage.Traverse():
        # Check if the prim is part of the specified marker group
        if prim.IsValid() and marker_name in str(prim.GetPath()):
            # Check if the prim's name starts with the rball prefix
            prim_name = prim.GetPath().name
            if prim_name.startswith(rball_prefix):
                try:
                    # Get world transform matrix
                    xform = omni.usd.get_world_transform_matrix(prim)

                    # Extract position from transform matrix
                    position = xform.ExtractTranslation()

                    # Store the position
                    rball_positions.append({
                        'path': str(prim.GetPath()),
                        'position': [position[0], position[1], position[2]]
                    })
                except Exception as e:
                    print(f"Could not get position for {prim.GetPath()}: {e}")

    return rball_positions

def on_raycast_hit(ray, result):
    global raycast_results
    if result.valid:
        raycast_results.append({
            'path': result.get_target_usd_path(),
            'position': result.hit_position,
            'normal': result.normal,
            'distance': result.hit_t
        })

def get_current_lines():
    """Get current line start and end positions"""
    object_positions = get_marker_rball_positions()
    start_point = [10, 0, 10]
    line_starts = [start_point] * len(object_positions)
    line_ends = [obj['position'] for obj in object_positions]
    return line_starts, line_ends

def update_lines():
    line_starts, line_ends = get_current_lines()
    colors = [
        [1, 0, 0, 1],    # Red
        [0, 1, 0, 1],    # Green
        [0, 0, 1, 1],    # Blue
        [1, 1, 0, 1]     # Yellow
    ]
    sizes = [5] * len(line_starts)
    
    draw.clear_lines()
    draw.draw_lines(line_starts, line_ends, colors, sizes)
    return line_starts, line_ends

def perform_raycast_queries():
    line_starts, line_ends = get_current_lines()
    for start, end in zip(line_starts, line_ends):
        direction = [end[i] - start[i] for i in range(3)]
        ray = omni.kit.raycast.query.Ray(tuple(start), tuple(direction))
        raycast.submit_raycast_query(ray, on_raycast_hit)

# Example function to print rball positions
def main():
    try:
        # Retrieve rball positions from the scene
        rball_positions = get_marker_rball_positions(marker_name="Marker", rball_prefix="rball_")

        # Print results
        if rball_positions:
            print("RBall Positions:")
            for rball in rball_positions:
                print(f"Path: {rball['path']}, Position: {rball['position']}")
        else:
            print("No rball objects found under the marker.")

    except Exception as e:
        print(f"Error: {e}")

# Run the script
if __name__ == "__main__":
    main()

# Add some example cubes
world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube1",
        name="cube1",
        position=np.array([0, -3, 0.6]),
        scale=np.array([1, 1, 1]),
        color=np.array([0.7, 0.5, 1.0])
    )
)

world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube2",
        name="cube2",
        position=np.array([2, 3, 0.6]),
        scale=np.array([1, 1, 1]),
        color=np.array([1.0, 0.5, 0.5])
    )
)


# Import the USD file at the specified position
import_usd_model(USD_FILE_PATH, np.array([18, 10, 5]))

# Simulation loop
try:
    frame_count = 0
    while simulation_app.is_running():
        # Reset raycast results each frame
        raycast_results = []
        
        # Step simulation and update
        world.step(render=True)
        line_starts, line_ends = update_lines()
        perform_raycast_queries()
        
        # Print combined results every 1000 frames
        if frame_count % 1000 == 0:
            print(f"\nFrame {frame_count}:")
            
            # Print object positions
            objects = get_marker_rball_positions()
            print("Ball positions:")
            for obj in objects:
                pos = np.round(obj['position'], decimals=3)
                print(f"  {obj['path']}: {pos}")
            
            # Print raycast results
            print("\nRaycast Results:")
            for result in raycast_results:
                pos = np.round(result['position'], decimals=3)
                normal = np.round(result['normal'], decimals=3)
                dist = round(result['distance'], 10)
                print(f"  Target: {result['path']}")
                print(f"  Hit at: {pos}")
                print(f"  Normal: {normal}")
                print(f"  Distance: {dist}\n")
                
        frame_count += 1

except KeyboardInterrupt:
    print("Simulation stopped by user")

finally:
    simulation_app.close()