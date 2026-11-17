"""Camera grid policies shared by geometric and semantic output."""

import numpy as np


def semantic_costs(grid):
    """Keep unknown cells unknown; classifier non-road labels are soft costs."""
    if grid.ndim != 2 or not grid.size or not np.isin(grid, (-1, 0, 100)).all():
        raise ValueError("Invalid semantic projection grid")
    return np.where(grid == 100, 40, grid).astype(np.int8)


def allow_blind_zone(costs, origin, resolution, base_from_odom, radius):
    """Treat only unknown cells in the body-forward half-disc as free."""
    if radius == 0:
        return costs
    rows = origin[1] + (np.arange(costs.shape[0])[:, None] + 0.5) * resolution
    cols = origin[0] + (np.arange(costs.shape[1])[None, :] + 0.5) * resolution
    x = base_from_odom[0, 0] * cols + base_from_odom[0, 1] * rows + base_from_odom[0, 3]
    y = base_from_odom[1, 0] * cols + base_from_odom[1, 1] * rows + base_from_odom[1, 3]
    result = costs.copy()
    result[(costs == -1) & (x >= 0) & (x * x + y * y <= radius * radius)] = 0
    return result
