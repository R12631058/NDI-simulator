import omni.usd
import carb
from omni.debugdraw import get_debug_draw_interface
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Usd

class TriggerVolumeDetector:
    def __init__(self, world, trigger_paths=None):
        self.world = world  # 使用外部傳入的 World 對象
        self.stage = omni.usd.get_context().get_stage()
        self.debug_draw = get_debug_draw_interface()
        self._box_size = Gf.Vec3f(100.0)    
        self._box_pos = Gf.Vec3f(0.0, 0.0, 0.0)
        
        # 默認觸發盒，或使用傳入的路徑
        default_trigger_path = "/World/Marker/surgery_room/NDI/NDI_mesh/VegaST_XT_cam/node_/volume"
        self.triggers = {
            "volume": {
                "path": default_trigger_path if trigger_paths is None else trigger_paths.get("volume", default_trigger_path),
                "objects": {},
                "previous_collisions": []
            }
        }
        self.frame_counter = 0
        self.print_interval = 30
        
        # 初始化觸發盒
        self._setup_triggers()
        
        self.previous_trigger_collisions = []
    
    def _setup_triggers(self):
        """設置所有觸發盒"""
        for trigger_name, trigger_info in self.triggers.items():
            trigger_path = trigger_info["path"]
            trigger_prim = self.stage.GetPrimAtPath(trigger_path)
            
            if trigger_prim.IsValid():
                print(f"設置觸發盒 {trigger_name} at {trigger_path}")
                UsdPhysics.CollisionAPI.Apply(trigger_prim)
                PhysxSchema.PhysxTriggerAPI.Apply(trigger_prim)
                trigger_info["state_api"] = PhysxSchema.PhysxTriggerStateAPI.Apply(trigger_prim)
                print(f"PhysX Trigger APIs 應用到 {trigger_name}")
            else:
                print(f"警告：觸發盒路徑 {trigger_path} 無效")
                
    def handle_trigger_event(self, trigger_name, other_prim_path, event_type):
        """處理觸發事件"""
        stage = self.stage
        other_prim = stage.GetPrimAtPath(other_prim_path)
        trigger_info = self.triggers[trigger_name]
        
        if event_type == "EnterEvent":
            trigger_info["objects"][other_prim_path] = other_prim
        elif event_type == "LeaveEvent":
            if other_prim_path in trigger_info["objects"]:
                del trigger_info["objects"][other_prim_path]
    
    def check_trigger_states(self):
        """檢查所有觸發盒狀態"""
        for trigger_name, trigger_info in self.triggers.items():
            if "state_api" not in trigger_info:
                continue
                
            current_collisions = trigger_info["state_api"].GetTriggeredCollisionsRel().GetTargets()
            previous_collisions = trigger_info["previous_collisions"]
            
            for collision in current_collisions:
                if collision not in previous_collisions:
                    self.handle_trigger_event(trigger_name, collision, "EnterEvent")
            
            for collision in previous_collisions:
                if collision not in current_collisions:
                    self.handle_trigger_event(trigger_name, collision, "LeaveEvent")
            
            trigger_info["previous_collisions"] = current_collisions
    
    def get_objects_in_trigger(self, trigger_name="volume"):
        """獲取指定觸發盒中的物體"""
        if trigger_name in self.triggers:
            return self.triggers[trigger_name]["objects"]
        return {}
    
    def print_all_trigger_contents(self):
        """顯示所有觸發盒內容"""
        for trigger_name, trigger_info in self.triggers.items():
            objects = trigger_info["objects"]
            print("\n" + "-"*70)
            print(f"{trigger_name}: {len(objects)} 個物體在觸發盒內")
            if len(objects) > 0:
                for index, (path, prim) in enumerate(objects.items(), 1):
                    print(f"{index}.{path}")
            else:
                print("觸發盒內沒有物體")
            print("-"*70)