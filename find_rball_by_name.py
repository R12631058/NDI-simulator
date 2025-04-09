# This scripts is excuted in the Isaac sim python terminal to find the rball objects under a marker group. 
import omni

def get_marker_rball_positions(marker_name="Marker", rball_prefix="rball_"):
    """
    Get positions of rball objects under the specified marker in the scene.

    :param marker_name: The name of the marker group to look for.
    :param rball_prefix: The prefix of the rball objects to filter.
    :return: A list of positions of the rball objects.
    """
    # Get the current stage
    stage = omni.usd.get_context().get_stage()

    # List to store rball positions
    rball_positions = []

    # Iterate through all prims in the stage
    for prim in stage.Traverse():
        # Check if the prim is part of the specified marker group
        if prim.IsValid() and marker_name in str(prim.GetPath()):
            # Check if the prim's name starts with the rball prefix
            prim_name = prim.GetPath().name
            if prim_name.startswith(rball_prefix):
                try:
                    # Get world transform matrix
                    xform = omni.usd.get_world_transform_matrix(prim)

                    # Extract position from transform matrix
                    position = xform.ExtractTranslation()

                    # Store the position
                    rball_positions.append({
                        'path': str(prim.GetPath()),
                        'position': [position[0], position[1], position[2]]
                    })
                except Exception as e:
                    print(f"Could not get position for {prim.GetPath()}: {e}")

    return rball_positions

# Example function to print rball positions
def main():
    try:
        # Retrieve rball positions from the scene
        rball_positions = get_marker_rball_positions(marker_name="Marker", rball_prefix="rball_")

        # Print results
        if rball_positions:
            print("RBall Positions:")
            for rball in rball_positions:
                print(f"Path: {rball['path']}, Position: {rball['position']}")
        else:
            print("No rball objects found under the marker.")

    except Exception as e:
        print(f"Error: {e}")

# Run the script
if __name__ == "__main__":
    main()
