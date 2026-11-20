"""Load one mandatory machine-local ground reference for all perception consumers."""
import json
import math
from pathlib import Path


def load_ground_reference(config, base):
    name = config.get("ground_height_file")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("ground_height_file is required; configure a Git-ignored local JSON file")
    path = (Path(base) / Path(name).expanduser()).resolve()
    try:
        value = json.loads(path.read_text())
    except OSError as error:
        raise ValueError(f"Cannot read ground_height_file {path}; supply the measured ground_z_m") from error
    except ValueError as error:
        raise ValueError("ground_height_file must contain valid JSON") from error
    if not isinstance(value, dict) or "ground_z_m" not in value:
        raise ValueError("ground_height_file must contain ground_z_m")
    ground = value["ground_z_m"]
    if type(ground) not in (int, float) or not math.isfinite(ground) or ground >= 0:
        raise ValueError("Local ground_z_m must be a finite negative base_link height, with no default")
    for section, field in (("terrain", "ground_z_m"), ("policy", "camera_expected_ground_z_m")):
        if field in config.get(section, {}):
            raise ValueError(f"Do not set {section}.{field} inline; use ground_height_file only")
    return str(path), float(ground)
