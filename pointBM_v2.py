from isaacsim import SimulationApp
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

class MarkerSimulation:
    def __init__(self):
        # Constants
        self.USD_FILE_PATH = "file://C:/Nick/surgery_team/surgery_team/BM.usd"
        
        # Initialize world and scene
        self.world = World()
        self.world.scene.add_default_ground_plane()
        
        # Initialize interfaces
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.raycast = omni.kit.raycast.query.acquire_raycast_query_interface()
        
        # Initialize state variables
        self.line_starts = []
        self.line_ends = []
        self.raycast_results = []
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
        self.import_usd_model(self.USD_FILE_PATH, np.array([18, 10, 5]))

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

    def on_raycast_hit(self, ray, result):
        if result.valid:
            self.raycast_results.append({
                'path': result.get_target_usd_path(),
                'position': result.hit_position,
                'normal': result.normal,
                'distance': result.hit_t
            })

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

    def get_current_lines(self):
        object_positions = self.get_marker_rball_positions()
        start_point = [10, 0, 10]
        self.line_starts = [start_point] * len(object_positions)
        self.line_ends = [obj['position'] for obj in object_positions]
        return self.line_starts, self.line_ends

    def update_lines(self):
        self.line_starts, self.line_ends = self.get_current_lines()
        colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 1]]
        sizes = [5] * len(self.line_starts)
        
        self.draw.clear_lines()
        self.draw.draw_lines(self.line_starts, self.line_ends, colors, sizes)
        return self.line_starts, self.line_ends

    def perform_raycast_queries(self):
        for start, end in zip(self.line_starts, self.line_ends):
            direction = [end[i] - start[i] for i in range(3)]
            ray = omni.kit.raycast.query.Ray(tuple(start), tuple(direction))
            self.raycast.submit_raycast_query(ray, self.on_raycast_hit)

    def print_frame_info(self):
        print(f"\nFrame {self.frame_count}:")
        objects = self.get_marker_rball_positions()
        print("Ball positions:")
        for obj in objects:
            pos = np.round(obj['position'], decimals=3)
            print(f"  {obj['path']}: {pos}")
        
        print("\nRaycast Results:")
        for result in self.raycast_results:
            pos = np.round(result['position'], decimals=3)
            normal = np.round(result['normal'], decimals=3)
            dist = round(result['distance'], 10)
            print(f"  Target: {result['path']}")
            print(f"  Hit at: {pos}")
            print(f"  Normal: {normal}")
            print(f"  Distance: {dist}\n")

    def run(self):
        try:
            while simulation_app.is_running():
                self.raycast_results = []
                self.world.step(render=True)
                self.line_starts, self.line_ends = self.update_lines()
                self.perform_raycast_queries()
                
                if self.frame_count % 1000 == 0:
                    self.print_frame_info()
                    
                self.frame_count += 1
                
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            simulation_app.close()

def main():
    sim = MarkerSimulation()
    sim.run()

if __name__ == "__main__":
    main()