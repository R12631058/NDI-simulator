# In v3, I new a sytem to determine the signal of the rball, and the signal is the hit state of the rball(0/1).
from isaacsim import SimulationApp
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Simulation started")

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
import omni.usd
import carb
import omni
from pxr import Gf, Usd, UsdGeom

class MarkerSimulation:
    def __init__(self):
        # Constants
        self.USD_FILE_PATH = "file://C:/Nick/surgery_team/surgery_team/BM.usd" 
        
        # Initialize world and scene
        self.world = World()
        self.world.scene.add_default_ground_plane()
        
        self.frame_count = 0
        
        # Add initial scene objects
        self._setup_scene()
        
    def _setup_scene(self):
        # Add example cubes
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/cube1",
                name="cube1",
                position=np.array([0, -3, 0.6]),
                scale=np.array([1, 1, 1]),
                color=np.array([0.7, 0.5, 1.0])
            )
        )
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/cube2",
                name="cube2",
                position=np.array([2, 3, 0.6]),
                scale=np.array([1, 1, 1]),
                color=np.array([1.0, 0.5, 0.5])
            )
        )
        # Import USD model
        self.import_usd_model(self.USD_FILE_PATH, np.array([18, 10, 15]))

    def import_usd_model(self, file_path, position):
        try:
            prim_path = "/World/Marker"
            add_reference_to_stage(file_path, prim_path)
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            
            if prim.IsValid():
                xform_op = prim.GetAttribute('xformOp:translate')
                if xform_op:
                    xform_op.Set(position)
                print(f"USD model imported successfully at {position}")
                return prim
            print("Failed to add USD model to stage")
            return None
        except Exception as e:
            print(f"Error importing USD model: {e}")
            return None

    
    def get_marker_rball_positions(self, marker_name="Marker", rball_prefix="rball_"):
        stage = omni.usd.get_context().get_stage()
        rball_positions = []
        
        for prim in stage.Traverse():
            if prim.IsValid() and marker_name in str(prim.GetPath()):
                prim_name = prim.GetPath().name
                if prim_name.startswith(rball_prefix):
                    try:
                        xform = omni.usd.get_world_transform_matrix(prim)
                        position = xform.ExtractTranslation()
                        rball_positions.append({
                            'path': str(prim.GetPath()),
                            'position': [position[0], position[1], position[2]]
                        })
                    except Exception as e:
                        print(f"Could not get position for {prim.GetPath()}: {e}")
        return rball_positions
    
    def print_frame_info(self):
        print(f"\nFrame {self.frame_count}:")
        objects = self.get_marker_rball_positions()
        print("Ball positions:")
        for obj in objects:
            pos = np.round(obj['position'], decimals=3)
            print(f"  {obj['path']}: {pos}") 
    
    def run(self):
        try:
            while simulation_app.is_running():
                
                self.world.step(render=True)

                if self.frame_count % 500 == 0:
                    self.print_frame_info()
                        
                self.frame_count += 1
                
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            simulation_app.close()

    def calculate_mean_position(positions):
        """Calculate mean position of all rballs"""
        if not positions:
            return None
            
        # Extract just the position arrays
        position_arrays = [obj['position'] for obj in positions]
        
        # Convert to numpy array and calculate mean
        positions_array = np.array(position_arrays)
        mean_pos = np.mean(positions_array, axis=0)
        
        return np.round(mean_pos, decimals=3)

    def print_frame_info(self):
        print(f"\nFrame {self.frame_count}:")
        objects = self.get_marker_rball_positions()
        print("Ball positions:")
        for obj in objects:
            pos = np.round(obj['position'], decimals=3)
            print(f"  {obj['path']}: {pos}")
        
        # Calculate and print mean position
        mean_pos = self.calculate_mean_position(objects)
        if mean_pos is not None:
            print(f"\nMean position of all balls: {mean_pos}")

def main():
    sim = MarkerSimulation()
    sim.run()

if __name__ == "__main__":
    main()