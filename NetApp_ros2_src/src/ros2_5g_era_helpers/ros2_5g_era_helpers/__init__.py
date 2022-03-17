import os

DEBUG=True

def get_path_to_assets() -> str:
    """
    Gets path to assets folder from environment variable.
    """

    

    path_to_assets = os.getenv("ROS2_5G_ERA_ASSETS_PATH")

    if not path_to_assets:
        if DEBUG:
            print("Environment variable ROS2_5G_ERA_ASSETS_PATH not set.")
            path_to_assets = os.path.abspath("assets")
            print(f"Trying to set path to: {path_to_assets}")
        else:
            raise EnvironmentError("Path to assets not set!")

    if not os.path.exists(path_to_assets):
        raise EnvironmentError("Path to assets does not exist!")

    return path_to_assets
