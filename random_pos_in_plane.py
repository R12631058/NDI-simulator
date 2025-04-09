#replicator: random position on plane
#mode: build in
import omni.replicator.core as rep

plane_path = "/World/Plane"
cube_path = "/World/Cube"
sphere_path = "/World/Sphere"



with rep.new_layer():
    with rep.trigger.on_time(interval = 1):
        traversable_plane = rep.get.prim_at_path(plane_path)

        cube = rep.get.prim_at_path(cube_path)
        sphere = rep.get.prim_at_path(sphere_path)
        
        with rep.utils.sequential():
            with sphere:
                rep.randomizer.scatter_2d(traversable_plane, check_for_collisions=True)
            with cube:
                rep.randomizer.scatter_2d(traversable_plane, no_coll_prims = [sphere], check_for_collisions=True)