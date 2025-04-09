#測試raycast_all的功能
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Continuous Raycast Monitor Started")

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.debug_draw import _debug_draw
import numpy as np
import omni.physx
import omni.usd
import time

class ContinuousRaycastMonitor:
    def __init__(self, num_cubes=11):
        # 創建世界和基本環境
        self.world = World()
        self.world.scene.add_default_ground_plane()
        
        # 初始化繪圖接口
        self.draw = _debug_draw.acquire_debug_draw_interface()
        
        # 設定參數
        self.num_cubes = num_cubes
        self.cube_paths = []
        self.cubes = []
        self.frame_count = 0
        self.update_interval = 60  # 每60幀更新一次
        
        # 創建立方體
        self._create_cubes()
        
        # 重置模擬
        self.world.reset()
        
        # 物理查詢接口
        self.physx_query = omni.physx.get_physx_scene_query_interface()
        
        # 等待物理穩定
        for _ in range(10):
            self.world.step(render=True)
        
        print("\n------------- 立方體資訊 -------------")
        for i, path in enumerate(self.cube_paths):
            pos = self.get_prim_position(path)
            if pos:
                print(f"Cube_{i} 位置: {np.round(pos, 3)}")
        
        print("\n------------- 持續射線監測啟動 -------------")
        print("監測器已啟動，正在持續監測從cube_0到cube_10的射線碰撞情況")
    
    def _create_cubes(self):
        """創建多個立方體"""
        for i in range(self.num_cubes):
            cube_path = f"/World/cube_{i}"
            self.cube_paths.append(cube_path)
            
            # 計算位置 - 沿著X軸每隔0.5m放置一個立方體
            pos_x = i * 0.5
            
            # 創建立方體
            cube = DynamicCuboid(
                prim_path=cube_path,
                name=f"cube_{i}",
                position=np.array([pos_x, 0, 0.5]),  # 高度為0.5m
                size=0.2,
                color=np.array([i/self.num_cubes, 0.5, 1-i/self.num_cubes]),  # 漸變顏色
            )
            self.cubes.append(cube)
    
    def get_prim_position(self, prim_path):
        """獲取物體位置"""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        
        if prim.IsValid():
            try:
                xform = omni.usd.get_world_transform_matrix(prim)
                position = xform.ExtractTranslation()
                return [position[0], position[1], position[2]]
            except Exception as e:
                print(f"無法獲取位置 {prim_path}: {e}")
                return None
        return None
    
    def process_hit_report(self, hit, hit_list):
        """處理射線碰撞報告"""
        hit_list.append({
            'rigid_body': hit.rigid_body,
            'position': hit.position,
            'distance': hit.distance
        })
        return True  # 繼續處理更多碰撞
    
    def perform_raycast(self):
        """執行從cube_0到cube_10的射線檢測"""
        # 獲取起點和終點的最新位置
        start_path = self.cube_paths[0]
        end_path = self.cube_paths[-1]
        
        start_pos = self.get_prim_position(start_path)
        end_pos = self.get_prim_position(end_path)
        
        if start_pos and end_pos:
            # 計算方向和距離
            direction = [end_pos[i] - start_pos[i] for i in range(3)]
            length = np.sqrt(sum([d*d for d in direction]))
            normalized_dir = [d / length for d in direction]
            
            # 畫線可視化射線
            self.draw.clear_lines()
            self.draw.draw_lines(
                [start_pos],
                [end_pos],
                [[0, 1, 0, 1]],  # 綠色
                [5]  # 線寬
            )
            
            # 收集射線碰撞
            hit_list = []
            max_distance = length * 1.1
            
            # 執行raycast_all
            self.physx_query.raycast_all(start_pos, normalized_dir, max_distance, 
                                        lambda hit: self.process_hit_report(hit, hit_list))
            
            # 返回碰撞列表
            return hit_list, start_pos, end_pos
        
        return None, None, None
    
    def print_hit_info(self, hit_list, start_pos, end_pos):
        """顯示射線碰撞信息"""
        if hit_list:
            # 清屏效果
            print("\033c", end="")  # 清屏命令
            
            print(f"\n===== 射線碰撞更新 (幀 {self.frame_count}) =====")
            print(f"從 cube_0 ({np.round(start_pos, 2)}) 到 cube_10 ({np.round(end_pos, 2)})")
            print(f"檢測到 {len(hit_list)} 個物體\n")
            
            # 按距離排序
            sorted_hits = sorted(hit_list, key=lambda x: x['distance'])
            
            # 顯示所有碰撞
            print("射線碰撞順序:")
            for i, hit in enumerate(sorted_hits):
                hit_path = hit['rigid_body']
                hit_dist = round(hit['distance'], 3)
                
                # 找出這是哪個立方體
                cube_idx = None
                for j, path in enumerate(self.cube_paths):
                    if path in hit_path:
                        cube_idx = j
                        break
                
                if cube_idx is not None:
                    print(f" {i+1}. cube_{cube_idx} ({hit_path}) - 距離: {hit_dist}")
                else:
                    print(f" {i+1}. {hit_path} - 距離: {hit_dist}")
            
            # 檢查是否所有立方體都被檢測到
            detected_indices = []
            for hit in sorted_hits:
                for j, path in enumerate(self.cube_paths):
                    if path in hit['rigid_body']:
                        detected_indices.append(j)
            
            detected_indices = sorted(list(set(detected_indices)))
            all_indices = list(range(self.num_cubes))
            missing = [i for i in all_indices if i not in detected_indices]
            
            print("\n摘要:")
            print(f"已檢測到的立方體: {detected_indices}")
            if missing:
                print(f"未檢測到的立方體: {missing}")
            else:
                print("所有立方體都被檢測到!")
    
    def run(self):
        """運行持續監測"""
        try:
            while simulation_app.is_running():
                self.world.step(render=True)
                self.frame_count += 1
                
                # 每隔一定幀數執行一次射線檢測並更新結果
                if self.frame_count % self.update_interval == 0:
                    hit_list, start_pos, end_pos = self.perform_raycast()
                    if hit_list:
                        self.print_hit_info(hit_list, start_pos, end_pos)
                
                time.sleep(0.02)  # 限制幀率
                
        except KeyboardInterrupt:
            print("\n使用者結束程式")
        finally:
            simulation_app.close()

def main():
    print("\n==== 持續射線監測工具 ====")
    print("此工具將在場景中創建11個立方體並持續監測射線碰撞情況")
    
    monitor = ContinuousRaycastMonitor()
    monitor.run()

if __name__ == "__main__":
    main()