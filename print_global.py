#Print global position of a USD prim in USD stage
import omni.usd

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/YourObject")  

global_transform = omni.usd.get_world_transform_matrix(prim)
print("Global Position:", global_transform.ExtractTranslation())
