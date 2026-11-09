"""Inspection of deploy-owned independent sensor layers after validation."""


def sensor_layers(runtime, costmap="global_costmap"):
    params = runtime["nav2_params"][costmap][costmap]["ros__parameters"]
    return {name: params[name] for name in params["plugins"]
            if name in ("lidar_layer", "camera_layer")}
