import omni
import omni.kit.app
import omni.kit.raycast.query
import numpy as np
from omni.isaac.debug_draw import _debug_draw
from pxr import UsdGeom, Gf

class RaycastVisualizer:
    def __init__(self):
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.raycast = omni.kit.raycast.query.acquire_raycast_query_interface()

        self.line_starts = []
        self.line_ends = []
        self.raycast_results = []

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
        """get given prim's position"""
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
        """get all Raycast start and end positions"""
        self.line_starts.clear()
        self.line_ends.clear()

        print(f"[Debug] Getting positions for start prims: {self.start_prim_paths}")  # 新增除錯訊息
        start_positions = [self.get_prim_position(path) for path in self.start_prim_paths]
        print(f"[Debug] Start positions: {start_positions}")  # 新增除錯訊息

        print(f"[Debug] Getting positions for end prims: {self.end_prim_paths}")  # 新增除錯訊息
        end_positions = [self.get_prim_position(path) for path in self.end_prim_paths]
        print(f"[Debug] End positions: {end_positions}")  # 新增除錯訊息

        start_positions = [pos for pos in start_positions if pos]
        end_positions = [pos for pos in end_positions if pos]

        if not start_positions or not end_positions:
            print("[Warning] Can't find start or end positions")
            return False

        # Generate lines from each start point to each end point
        for start_pos in start_positions:
            for end_pos in end_positions:
                print(f"[Debug] Adding line from {start_pos} to {end_pos}")  # 新增除錯訊息
                self.line_starts.append(start_pos)
                self.line_ends.append(end_pos)

        return True

    def perform_raycast(self):
        """Execute Raycast and store results"""
        try:
            self.raycast_results.clear()

            for start, end in zip(self.line_starts, self.line_ends):
                try:
                    direction = [end[i] - start[i] for i in range(3)]
                    magnitude = sum(d*d for d in direction) ** 0.5
                    
                    if magnitude == 0:
                        print("[Warning] Zero direction vector detected")
                        continue
                        
                    # 標準化方向向量
                    direction = [d/magnitude for d in direction]
                    
                    print(f"[Debug] Raycast from {start} in direction {direction}")
                    ray = omni.kit.raycast.query.Ray(tuple(start), tuple(direction))
                    self.raycast.submit_raycast_query(ray, self.on_raycast_hit)
                    
                except Exception as e:
                    print(f"[Error] Error in raycast calculation: {str(e)}")
                    continue
                    
        except Exception as e:
            print(f"[Error] Error in perform_raycast: {str(e)}")

    def on_raycast_hit(self, ray, result):
        """Raycast result callback"""
        if result.valid:
            self.raycast_results.append({
                'path': result.get_target_usd_path(),
                'position': result.hit_position,
                'normal': result.normal,
                'distance': result.hit_t
            })

    def visualize_raycast(self):
        """Visualize the raycast lines"""
        try:
            if not self.line_starts or not self.line_ends:
                return

            # 每次更新前清除舊的線條
            self.draw.clear_lines()
            
            colors = [[1, 0, 0, 1]] * len(self.line_starts)  # 紅色線條
            sizes = [3] * len(self.line_starts)  # 線條寬度

            self.draw.draw_lines(self.line_starts, self.line_ends, colors, sizes)
        except Exception as e:
            print(f"[Error] Error in visualize_raycast: {str(e)}")

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
            # 限制迴圈執行次數
            max_iterations = 10
            iteration = 0
            
            while omni.kit.app.get_app().is_running() and iteration < max_iterations:
                print(f"[Debug] Iteration {iteration + 1}")
                
                try:
                    if self.update_lines():
                        print("[Debug] Lines updated successfully")
                        print("[Debug] Starting perform_raycast()")
                        self.perform_raycast()
                        print("[Debug] Starting visualize_raycast()")
                        self.visualize_raycast()

                    if self.raycast_results:
                        self.print_raycast_results()
                        
                    # 增加短暫延遲，避免過度頻繁的更新
                    import time
                    time.sleep(0.1)
                    
                except Exception as inner_e:
                    print(f"[Error] Error in iteration {iteration}: {str(inner_e)}")
                    
                
                iteration += 1
                    
        except Exception as e:
            print(f"[Error] An error occurred in main loop: {str(e)}")
            
        except KeyboardInterrupt:
            print("User stopped the simulation")
        finally:
            print(f"Raycast Visualizer stopped after {iteration} iterations")

def main():
    visualizer = RaycastVisualizer()
    visualizer.run()

if __name__ == "__main__":
    main()
