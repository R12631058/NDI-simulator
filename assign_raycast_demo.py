#this standalone script import a USD model and create a raycast from the model to the other model
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Simulation started")

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.debug_draw import _debug_draw
import numpy as np
import carb
import omni
import omni.kit.raycast.query
from pxr import Gf, Usd, UsdGeom

class InteractiveRaycast:
    def __init__(self):
        self.world = World()
        self.world.scene.add_default_ground_plane()

        self.USD_FILE_PATH = "C:/Nick/surgery_team/surgery_team/USD/FourFill_condition.usd"
        #"C:/Nick/surgery_team/surgery_team/USD/surgery_room.usd"

        # Initialize interfaces
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.raycast = omni.kit.raycast.query.acquire_raycast_query_interface()
        
        # Initialize state variables
        self.line_starts = []
        self.line_ends = []
        self.raycast_results = []
        self.frame_count = 0
        
        # Setup initial scene with cubes
        self._setup_scene()
        
        # Get user input for prims
        print("\nAvailable prims:")
        for i in range(1, 9):
            print(f"- /World/cube{i}")

        start_prims = input("Enter the prim paths for line starts (separate with commas, e.g. /World/cube1,/World/cube3): ")
        end_prims = input("Enter the prim paths for line ends (separate with commas, e.g. /World/cube2,/World/cube4): ")
        self.start_prim_paths = [path.strip() for path in start_prims.split(',')]
        self.end_prim_paths = [path.strip() for path in end_prims.split(',')]

    def import_usd_model(self, file_path, position):
        try:
            prim_path = "/World/Marker"
            add_reference_to_stage(file_path, prim_path)
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            
            if prim.IsValid():
                # Create or get the xformable API
                xformable = UsdGeom.Xformable(prim)
                
                # Clear any existing transform operations
                xformable.ClearXformOpOrder()
                
                # Add a new transform operation
                xform_op = xformable.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble)
                xform_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
                
                print(f"USD model imported successfully at {position}")
                return prim
            print("Failed to add USD model to stage")
            return None
        except Exception as e:
            print(f"Error importing USD model: {e}")
            return None     
            
    def _setup_scene(self):
        # Add example cubes
        self.import_usd_model(self.USD_FILE_PATH, np.array([0, 0, 0.3]))

    def get_prim_position(self, prim_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        
        if prim.IsValid():
            try:
                xform = omni.usd.get_world_transform_matrix(prim)
                position = xform.ExtractTranslation()
                return [position[0], position[1], position[2]]
            except Exception as e:
                print(f"Could not get position for {prim_path}: {e}")
                return None
        return None

    def update_lines(self):
        start_positions = []
        end_positions = []
        
        # Get all start positions
        for start_path in self.start_prim_paths:
            start_pos = self.get_prim_position(start_path)
            if start_pos:
                start_positions.append(start_pos)
        
        # Get all end positions
        for end_path in self.end_prim_paths:
            end_pos = self.get_prim_position(end_path)
            if end_pos:
                end_positions.append(end_pos)
        
        if start_positions and end_positions:
            # Create lines from each start point to each end point
            self.line_starts = []
            self.line_ends = []
            
            for start_pos in start_positions:
                for end_pos in end_positions:
                    self.line_starts.append(start_pos)
                    self.line_ends.append(end_pos)
            
            # Draw debug lines
            self.draw.clear_lines()
            colors = [[1, 0, 0, 1]] * len(self.line_starts)  # Red color for all lines
            sizes = [5] * len(self.line_starts)  # Line width
            self.draw.draw_lines(
                self.line_starts,
                self.line_ends,
                colors,
                sizes
            )
            return True
        return False

    def on_raycast_hit(self, ray, result):
        if result.valid:
            self.raycast_results.append({
                'path': result.get_target_usd_path(),
                'position': result.hit_position,
                'normal': result.normal,
                'distance': result.hit_t
            })

    def perform_raycast(self):
        if self.line_starts and self.line_ends:
            for start, end in zip(self.line_starts, self.line_ends):
                direction = [end[i] - start[i] for i in range(3)]
                ray = omni.kit.raycast.query.Ray(
                    tuple(start), 
                    tuple(direction)
                )
                self.raycast.submit_raycast_query(ray, self.on_raycast_hit)

    def print_raycast_info(self):
        print("\nRaycast Results:")
        for result in self.raycast_results:
            pos = np.round(result['position'], decimals=3)
            normal = np.round(result['normal'], decimals=3)
            dist = round(result['distance'], 3)
            print(f"  Target: {result['path']}")
            print(f"  Hit at: {pos}")
            print(f"  Normal: {normal}")
            print(f"  Distance: {dist}\n")

    def run(self):
        try:
            while simulation_app.is_running():
                self.raycast_results = []
                self.world.step(render=True)
                
                if self.update_lines():
                    self.perform_raycast()
                
                if self.frame_count % 100 == 0:  # Print every 100 frames
                    if self.raycast_results:
                        self.print_raycast_info()
                
                self.frame_count += 1
                
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            simulation_app.close()

def main():
    raycast = InteractiveRaycast()
    raycast.run()

if __name__ == "__main__":
    main()