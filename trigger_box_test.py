from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Simulation started")

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.usd
import time
import carb
import omni.timeline
from omni.debugdraw import get_debug_draw_interface
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, Usd

class TriggerVolumeDetector:
    def __init__(self):
        self.USD_FILE_PATH = "C:/Nick/surgery_team/surgery_team/USD/FourFill_condition.usd"
        self.world = World()
        self.stage = omni.usd.get_context().get_stage()
        self.debug_draw = get_debug_draw_interface()
        self._box_size = Gf.Vec3f(100.0)    
        self._box_pos = Gf.Vec3f(0.0, 0.0, 0.0)  
        self.triggers = {
            "volume": {
                "path": "/World/Triggerbox/surgery_room/NDI/NDI_mesh/VegaST_XT_cam/node_/volume",
                "objects": {},
                "previous_collisions": []
            }
        }
        self.frame_counter = 0
        self.print_interval = 30

        self._setup_scene()
        
        self.objects_in_trigger = {}
        
        self.previous_trigger_collisions = []
    
    def import_usd_model(self, file_path, position):
        try:
            prim_path = "/World/Triggerbox"
            add_reference_to_stage(file_path, prim_path)
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            
            if prim.IsValid():
                print(f"USD model imported successfully at {position}")
                return prim
            print("Failed to add USD model to stage")
            return None
        except Exception as e:
            print(f"Error importing USD model: {e}")
            return None

    def _setup_scene(self):
        self.import_usd_model(self.USD_FILE_PATH, [0, 0, 100])
        
        for trigger_name, trigger_info in self.triggers.items():
            trigger_path = trigger_info["path"]
            trigger_prim = self.stage.GetPrimAtPath(trigger_path)
            
            if trigger_prim.IsValid():
                UsdPhysics.CollisionAPI.Apply(trigger_prim)
                PhysxSchema.PhysxTriggerAPI.Apply(trigger_prim)
                trigger_info["state_api"] = PhysxSchema.PhysxTriggerStateAPI.Apply(trigger_prim)
                print(f"PhysX Trigger APIs applied to {trigger_name}")

    def handle_trigger_event(self, trigger_name, other_prim_path, event_type):
        stage = self.stage
        other_prim = stage.GetPrimAtPath(other_prim_path)
        trigger_info = self.triggers[trigger_name]
        
        if event_type == "EnterEvent":
            trigger_info["objects"][other_prim_path] = other_prim
        elif event_type == "LeaveEvent":
            if other_prim_path in trigger_info["objects"]:
                del trigger_info["objects"][other_prim_path]
    
    def print_trigger_contents(self):
        print("\n" + "-"*70)
        if len(self.objects_in_trigger) > 0:
            print(f"{len(self.objects_in_trigger)} prims in trigger box:")
            for index, (path, prim) in enumerate(self.objects_in_trigger.items(), 1):
                print(f"{index}.{path}")
        else:
            print("No prims in trigger box")
        print("-"*70 + "\n")

    def check_trigger_states(self):
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

    def get_objects_in_trigger(self):
        return self.objects_in_trigger    

    def print_objects_in_trigger(self):
        print("\n" + "-"*50)
        print(f"Prims in volume ({len(self.objects_in_trigger)}):")
        for path, prim in self.objects_in_trigger.items():
            print(f" - {path} | type: {prim.GetTypeName()}")
        print("-"*50 + "\n")        

    def print_all_trigger_contents(self):
        for trigger_name, trigger_info in self.triggers.items():
            objects = trigger_info["objects"]
            print("\n" + "-"*70)
            print(f"{trigger_name}: {len(objects)} prims in trigger")
            if len(objects) > 0:
                for index, (path, prim) in enumerate(objects.items(), 1):
                    print(f"{index}.{path}")
            else:
                print("No prims in this trigger")
            print("-"*70)

def main():
    trigger_volume_detector = TriggerVolumeDetector()
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    
    print("\nProgram is running. Press Ctrl+C in terminal to exit.")
    try:
        while True:
            simulation_app.update()
            #trigger_volume_detector._draw_trigger_box()
            trigger_volume_detector.check_trigger_states()
            
            trigger_volume_detector.frame_counter += 1
            if trigger_volume_detector.frame_counter >= trigger_volume_detector.print_interval:
                trigger_volume_detector.print_all_trigger_contents()
                trigger_volume_detector.frame_counter = 0
                
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        simulation_app.close()

if __name__ == "__main__":
    main()
