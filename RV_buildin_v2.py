# Description: This program will be run in Standalone In-Built Editor mode to test the Raycast functionimport omni
import carb
import omni
import omni.kit.app
import omni.kit.raycast.query
import numpy as np
from omni.isaac.debug_draw import _debug_draw
from pxr import UsdGeom, Gf
from omni.isaac.core.simulation_context import SimulationContext

class RaycastVisualizer:
    def __init__(self):
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.raycast = omni.kit.raycast.query.acquire_raycast_query_interface()
        self.simulation_context = SimulationContext()

        self.line_starts = []
        self.line_ends = []
        self.raycast_results = []
        self.frame_count = 0

        #get the object name entered by the user
        self.get_user_input()

    def get_user_input(self):
        """user input for the object names"""
        stage = omni.usd.get_context().get_stage()

        print("\nobject available list：")
        # 修改遍歷方式，列出更多類型的物件
        prim_paths = []
        for prim in stage.Traverse():
            # 檢查常見的幾何體類型
            if (prim.IsA(UsdGeom.Xform) or 
                prim.IsA(UsdGeom.Cube) or 
                prim.IsA(UsdGeom.Sphere) or 
                prim.IsA(UsdGeom.Cylinder) or 
                prim.IsA(UsdGeom.Mesh) or 
                prim.IsA(UsdGeom.Cone)):
                prim_type = prim.GetTypeName()
                prim_path = prim.GetPath().pathString
                print(f"- {prim_path} ({prim_type})")
                prim_paths.append(prim_path)

        start_prims = input("\nInput Raycast start prim(ex:/World/Cube1,/World/Robot): ")
        end_prims = input("Input Raycast end point prim (ex:/World/Cube2,/World/Table): ")

        self.start_prim_paths = [path.strip() for path in start_prims.split(",")]
        self.end_prim_paths = [path.strip() for path in end_prims.split(",")]
    
    def get_prim_position(self, prim_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        if prim.IsValid():
            try:
                xform = omni.usd.get_world_transform_matrix(prim)
                position = xform.ExtractTranslation()
                return [position[0], position[1], position[2]]
            except Exception as e:
                print(f"[Error] Can't get {prim_path} position: {e}")
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

    def perform_raycast(self):
        if self.line_starts and self.line_ends:
            for start, end in zip(self.line_starts, self.line_ends):
                direction = [end[i] - start[i] for i in range(3)]
                ray = omni.kit.raycast.query.Ray(
                    tuple(start), 
                    tuple(direction)
                )
                self.raycast.submit_raycast_query(ray, self.on_raycast_hit)

    def on_raycast_hit(self, ray, result):
        """Raycast result callback"""
        if result.valid:
            self.raycast_results.append({
                'path': result.get_target_usd_path(),
                'position': result.hit_position,
                'normal': result.normal,
                'distance': result.hit_t
            })

    def print_raycast_results(self):
        print("\nRaycast Result：")
        for result in self.raycast_results:
            pos = np.round(result['position'], decimals=3)
            normal = np.round(result['normal'], decimals=3)
            dist = round(result['distance'], 3)
            print(f"  Hit object: {result['path']}")
            print(f"  Hit position: {pos}")
            print(f"  Normal: {normal}")
            print(f"  Distance: {dist}\n")

    def run(self):
        try:
            # start simulation 
            while True:
                if self.simulation_context.is_playing():
                    self.raycast_results = []
                    self.simulation_context.step()
                    
                    if self.update_lines():
                        self.perform_raycast()
                    
                    if self.frame_count % 100 == 0:  # Print every 100 frames
                        if self.raycast_results:
                            self.print_raycast_results()
                    
                    self.frame_count += 1
                
                # 加入小延遲以避免 CPU 使用過高
                carb.events.sleep(0.01)
                
        except KeyboardInterrupt:
            print("Simulation stopped by user")
        finally:
            print("Simulation stopped")

def main():
    visualizer = RaycastVisualizer()
    visualizer.run()

if __name__ == "__main__":
    main()
