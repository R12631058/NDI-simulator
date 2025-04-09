from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
print("Simulation started")

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.usd
from pxr import Gf, UsdGeom

class TestUsdImport:
    def __init__(self):
        self.world = World()
        self.world.scene.add_default_ground_plane()
        self.USD_FILE_PATH = "C:/Nick/surgery_team/surgery_team/USD/FourFill_condition.usd"
        #"C:/Nick/surgery_team/surgery_team/3d_model/tm5-900_workstation/workstation.usd"
        #"C:/Nick/surgery_team/trigger_box.usd"
    def import_usd_model(self):
        try:
            
            prim_path = "/World/SurgeryRoom"
            print(f"Attempting to import USD from: {self.USD_FILE_PATH}")
            stage = omni.usd.get_context().get_stage()

            if not stage.GetPrimAtPath(prim_path):
                stage.DefinePrim(prim_path, "Xform")
            
            prim = stage.GetPrimAtPath(prim_path)
            prim.GetReferences().AddReference(self.USD_FILE_PATH)
            
            if prim.IsValid():
                print("USD model imported successfully!")
            
                xformable = UsdGeom.Xformable(prim)
                xformable.ClearXformOpOrder()
                xform_op = xformable.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble)
                xform_op.Set(Gf.Vec3d(0, 0, 0))  
                
                stage.Save()
                return True
            else:
                print("Failed to import USD model - Prim is not valid")
                return False
            
        except Exception as e:
            print(f"Error importing USD model: {str(e)}")
            return False

    def run(self):
        import_success = self.import_usd_model()
        
        if import_success:
            try:
                print("Running simulation - Press Ctrl+C to stop")
                while simulation_app.is_running():
                    self.world.step(render=True)
            except KeyboardInterrupt:
                print("Simulation stopped by user")
        else:
            print("Import failed - checking simulation")
            
        simulation_app.close()

def main():
    test = TestUsdImport()
    test.run()

if __name__ == "__main__":
    main()