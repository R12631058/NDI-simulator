import omni.kit.raycast.query
from pxr import Gf, UsdGeom

# get raycast interface
raycast = omni.kit.raycast.query.acquire_raycast_query_interface()

# set up a cube for test
stage = omni.usd.get_context().get_stage()
CUBE_PATH = "/Cube"
cube = UsdGeom.Cube.Define(stage, CUBE_PATH)
UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(123.45, 0, 0))

# generate a ray
ray = omni.kit.raycast.query.Ray((1000, 0, 0), (-1, 0, 0))

def callback(ray, result):
    if result.valid:
        # Got the raycast result in the callback
        print(Gf.Vec3d(*result.hit_position))
        print(result.hit_t)
        print(Gf.Vec3d(*result.normal))
        print(result.get_target_usd_path())

raycast.submit_raycast_query(ray, callback)