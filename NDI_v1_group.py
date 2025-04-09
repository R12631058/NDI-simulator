#this standalone script import a USD model and create a raycast from the model to the other model
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracedLighting",
    "physics_gpu": 0
})
print("Simulation started")
from trigger_box import TriggerVolumeDetector  # 導入我們剛建立的模組
import time
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.debug_draw import _debug_draw
import numpy as np
import carb
import omni
import omni.kit.raycast.query
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema

class InteractiveRaycast:
    def __init__(self):
        self.world = World()
        self.world.scene.add_default_ground_plane()

        self.USD_FILE_PATH = "C:/Nick/surgery_team/surgery_team/USD/FourFill_condition.usd"
        #"C:/Nick/surgery_team/surgery_team/USD/surgery_room.usd"
        self._list_available_prims()

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
        # 初始化觸發體積檢測器
        self.trigger_detector = self._setup_trigger_volume()

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
        
        # 等待物理穩定
        print("\n等待場景初始化...")
        for _ in range(10):
            self.world.step(render=True)

    def _setup_trigger_volume(self):
        """初始化觸發體積檢測器"""
        from omni.debugdraw import get_debug_draw_interface
        from pxr import PhysxSchema
        
        trigger_detector = {
            "triggers": {
                "volume": {
                    "path": "/World/Marker/surgery_room/NDI/NDI_mesh/VegaST_XT_cam/node_/volume",  # 根據您的USD結構調整路徑
                    "objects": {},
                    "previous_collisions": []
                }
            },
            "frame_counter": 0,
            "print_interval": 100  # 每100幀輸出一次
        }
        
        stage = omni.usd.get_context().get_stage()
        
        # 為每個觸發器應用物理觸發器API
        for trigger_name, trigger_info in trigger_detector["triggers"].items():
            trigger_path = trigger_info["path"]
            trigger_prim = stage.GetPrimAtPath(trigger_path)
            
            if trigger_prim.IsValid():
                UsdPhysics.CollisionAPI.Apply(trigger_prim)
                PhysxSchema.PhysxTriggerAPI.Apply(trigger_prim)
                trigger_info["state_api"] = PhysxSchema.PhysxTriggerStateAPI.Apply(trigger_prim)
                print(f"物理觸發器API已應用於 {trigger_name}")
            else:
                print(f"警告: 找不到觸發器物體於路徑 {trigger_path}")
        
        return trigger_detector
    
    def _handle_trigger_event(self, trigger_name, other_prim_path, event_type):
        """處理觸發器事件"""
        stage = omni.usd.get_context().get_stage()
        other_prim = stage.GetPrimAtPath(other_prim_path)
        trigger_info = self.trigger_detector["triggers"][trigger_name]
        
        if event_type == "EnterEvent":
            trigger_info["objects"][other_prim_path] = other_prim
        elif event_type == "LeaveEvent":
            if other_prim_path in trigger_info["objects"]:
                del trigger_info["objects"][other_prim_path]
                
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

    def _list_available_prims(self):
        """列出場景中所有可用的物體路徑"""
        print("\n掃描場景中的物體...")
        stage = omni.usd.get_context().get_stage()
        
        if not stage:
            print("錯誤：無法獲取場景舞台")
            return
        
        print("\n可用物體路徑:")
        for prim in stage.Traverse():
            if prim.IsValid():
                print(f"- {prim.GetPath()}")

    def _validate_paths(self):
        """驗證輸入的物體路徑是否有效"""
        print("\n驗證輸入的物體路徑...")
        stage = omni.usd.get_context().get_stage()
        
        all_valid = True
        
        # 檢查起點物體
        for path in self.start_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                print(f"警告：找不到起點物體 '{path}'")
                all_valid = False
        
        # 檢查終點物體
        for path in self.end_prim_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                print(f"警告：找不到終點物體 '{path}'")
                all_valid = False
        
        if all_valid:
            print("所有物體路徑驗證通過")
        else:
            print("警告：部分物體路徑無效，模擬可能無法正常進行")
            print("可能的解決方法：")
            print("1. 確認 USD 文件中包含了這些物體")
            print("2. 路徑格式應為 '/World/Marker/Cube_0' 格式")
            print("3. 注意大小寫和特殊字符")

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
        """處理射線碰撞回調，只專注於檢查擊中的物體路徑"""
        ray_index = len(self.raycast_results)
        
        # 如果超出範圍，則返回
        if ray_index >= len(self.ray_info):
            return
        
        # 獲取該射線的信息
        info = self.ray_info[ray_index]
        i = info['index']
        start_path = info['start_path']
        end_path = info['end_path']
        
        if result.valid:
            hit_path = result.get_target_usd_path()
            
            # 只列印射線編號、起終點和擊中物體
            #if self.frame_count % 100 == 0:
            #    print(f"Ray {i+1}: {start_path} → {end_path} hit: {hit_path}")
            
            # 儲存最小必要資訊
            self.raycast_results.append({
                'ray_index': i,
                'start_path': start_path, 
                'end_path': end_path,
                'hit_path': hit_path
            })
        else:
            if self.frame_count % 100 == 0:
                print(f"Ray {i+1}: {start_path} → {end_path} hit: None")
            
            self.raycast_results.append({
                'ray_index': i,
                'start_path': start_path,
                'end_path': end_path,
                'hit_path': "None"
            })

    def check_api(self):
        """檢查 raycast API 可用方法"""
        print("\n可用的射線檢測方法:")
        print(dir(self.raycast))

    def perform_raycast(self):
        if not self.line_starts or not self.line_ends:
            print("錯誤: 沒有線條可以射線檢測")
            return
        
        # 儲存射線信息，以便在回調中使用
        self.ray_info = []
        
        for i, (start, end) in enumerate(zip(self.line_starts, self.line_ends)):
            # 計算起點和終點索引
            start_path_idx = i // len(self.end_prim_paths)
            end_path_idx = i % len(self.end_prim_paths)
            
            start_path = self.start_prim_paths[start_path_idx]
            end_path = self.end_prim_paths[end_path_idx]
            
            # 計算方向向量
            direction = [end[j] - start[j] for j in range(3)]
            length = np.sqrt(sum([d*d for d in direction]))
            normalized_dir = [d / length for d in direction]
            
            # 添加微小偏移，避免自我碰撞
            offset = 0.01  # 1cm 偏移
            adjusted_start = [start[j] + normalized_dir[j] * offset for j in range(3)]
            
            # 創建射線
            ray = omni.kit.raycast.query.Ray(tuple(adjusted_start), tuple(normalized_dir))
            
            # 儲存射線信息
            self.ray_info.append({
                'index': i,
                'start_path': start_path,
                'end_path': end_path
            })
            
            # 提交異步查詢
            
            self.raycast.submit_raycast_query(ray, self.on_raycast_hit)

    def _check_trigger_states(self):
        """檢查所有觸發器的狀態"""
        for trigger_name, trigger_info in self.trigger_detector["triggers"].items():
            if "state_api" not in trigger_info:
                continue
                
            current_collisions = trigger_info["state_api"].GetTriggeredCollisionsRel().GetTargets()
            previous_collisions = trigger_info["previous_collisions"]
            
            for collision in current_collisions:
                if collision not in previous_collisions:
                    self._handle_trigger_event(trigger_name, collision, "EnterEvent")
            
            for collision in previous_collisions:
                if collision not in current_collisions:
                    self._handle_trigger_event(trigger_name, collision, "LeaveEvent")
            
            trigger_info["previous_collisions"] = current_collisions

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

    '''
    #For debugging purposes, to print all objects in the trigger volume
    def _print_all_trigger_contents(self):
        """輸出所有觸發器內的物件"""
        print("\n=== 觸發器體積中的物體 ===")
        for trigger_name, trigger_info in self.trigger_detector["triggers"].items():
            objects = trigger_info["objects"]
            print(f"觸發器 '{trigger_name}': {len(objects)} 個物體")
            
            if len(objects) > 0:
                for index, (path, prim) in enumerate(objects.items(), 1):
                    print(f"  {index}. {path} | 類型: {prim.GetTypeName()}")
            else:
                print("  無物體在此觸發器中")
        print("==============================")
    '''

    def process_raycast_hits(self):
        """檢查射線擊中狀態，並標示物體是否在觸發器內"""
        if self.frame_count % 100 == 0 and self.raycast_results:
            print("\n=== 射線目標擊中狀態 ===")
            
            # 先獲取所有在觸發器中的物體路徑集合，方便後續查詢
            objects_in_trigger = set()
            for trigger_name, trigger_info in self.trigger_detector["triggers"].items():
                for obj_path in trigger_info["objects"].keys():
                    objects_in_trigger.add(str(obj_path))  # 確保轉換為字符串
            
            # 建立終點路徑到結果的映射
            end_hit_map = {}
            
            for end_path in self.end_prim_paths:
                end_hit_map[end_path] = {
                    'hit_count': 0,
                    'total_count': 0
                }
            
            # 統計每個終點有多少射線擊中它
            for result in self.raycast_results:
                end_path = result['end_path']
                hit_path = result['hit_path']
                
                if end_path in end_hit_map:
                    end_hit_map[end_path]['total_count'] += 1
                    if hit_path != "None" and end_path in hit_path:
                        end_hit_map[end_path]['hit_count'] += 1
            
            # 顯示每個終點的擊中率、位置，以及是否在觸發器內
            visible_paths = []
            for end_path, stats in end_hit_map.items():
                hit_count = stats['hit_count']
                total = stats['total_count']
                
                # 使用更靈活的比較方式檢查物體是否在觸發器內
                is_in_trigger = False
                for obj_in_trigger in objects_in_trigger:
                    # 使用字符串包含關係來檢測，更靈活地處理路徑格式差異
                    if end_path in obj_in_trigger or obj_in_trigger in end_path:
                        is_in_trigger = True
                        break
                        
                trigger_status = "[In]" if is_in_trigger else "[Out]"
                
                # 只有當射線全部擊中且物體在觸發器內時，才顯示位置
                if hit_count == total and is_in_trigger:  
                    # 獲取物體位置
                    position = self.get_prim_position(end_path)
                    if position:
                        pos_str = f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]"
                        print(f"{end_path} {trigger_status}: {hit_count}/{total} 射線擊中 {pos_str}")
                        visible_paths.append(end_path)
                    else:
                        print(f"{end_path} {trigger_status}: {hit_count}/{total} 射線擊中 [位置未知]")
                else:
                    # 射線未全部擊中或物體不在觸發器內，顯示 Lost position
                    print(f"{end_path} {trigger_status}: {hit_count}/{total} 射線擊中 Lost position")
            
            # 總體統計
            total_hits = sum(stats['hit_count'] for stats in end_hit_map.values())
            total_rays = sum(stats['total_count'] for stats in end_hit_map.values())
            print(f"\n總計: {total_hits}/{total_rays} 射線擊中終點")
            
            # 如果有足夠可見球體，計算標記位置
            if len(visible_paths) >= 3:
                visible_positions = []
                for path in visible_paths:
                    pos = self.get_prim_position(path)
                    if pos:
                        visible_positions.append(pos)
                
                if visible_positions:
                    positions = np.array(visible_positions)
                    mean_pos = np.round(np.mean(positions, axis=0), decimals=3)
                    print(f"\n標記位置 ({len(visible_paths)} 個可見點): {mean_pos}")

                else:
                    print(f"\n可見球體: {len(visible_paths)}，lost marker position")   

    def run(self):
        try:
            print("\n=== 模擬啟動 ===")
            print("使用者輸入:")
            print(f"起點物體: {self.start_prim_paths}")
            print(f"終點物體: {self.end_prim_paths}")
            print("\n每 100 幀更新一次結果...")
            print("按 Ctrl+C 結束模擬\n")
            
            # 確保 _validate_paths 方法被調用
            self._validate_paths()
            
            # 設置計時器
            last_update_time = time.time()
            
            while simulation_app.is_running():
                # 每步驟顯示一個進度指示器
                if self.frame_count % 10 == 0:
                    progress = self.frame_count % 100
                    progress_bar = "[" + "#" * (progress // 5) + " " * (20 - progress // 5) + "]"
                    print(f"\r處理中 {progress_bar} 幀 {self.frame_count}", end="")
                
                # 步進模擬
                self.world.step(render=True)
                
                # 檢查觸發器狀態
                self._check_trigger_states()
                self.trigger_detector["frame_counter"] += 1

                if self.update_lines():
                    # 只在每 100 幀的時候進行射線檢測
                    if self.frame_count % 100 == 0:
                        self.raycast_results = []  # 清空上一次的結果
                        print("\n\n=== 執行射線檢測 (幀 {}) ===".format(self.frame_count))
                        
                        # 執行射線檢測
                        self.perform_raycast()
                        
                        # 等待足夠的時間讓射線檢測完成（異步回調）
                        wait_frames = 5
                        for _ in range(wait_frames):
                            self.world.step(render=True)
                        
                        # 處理結果
                        print(f"\n收到 {len(self.raycast_results)} 個射線檢測結果")
                        if len(self.raycast_results) > 0:
                            self.process_raycast_hits()
                        else:
                            print("警告: 未收到射線檢測結果，請檢查物體路徑")

                        # 輸出觸發器狀態
                        if self.trigger_detector["frame_counter"] >= self.trigger_detector["print_interval"]:
                            #self._print_all_trigger_contents()
                            self.trigger_detector["frame_counter"] = 0    
                
                self.frame_count += 1
                
                # 控制模擬速度
                time.sleep(0.01)  # 限制每秒約 100 幀
                    
        except Exception as e:
            print(f"\n\n模擬出現錯誤: {e}")
        except KeyboardInterrupt:
            print("\n\n使用者手動停止模擬")
        finally:
            print("\n模擬結束")
            simulation_app.close()

def main():
    raycast = InteractiveRaycast()
    raycast.run()

if __name__ == "__main__":
    main()


