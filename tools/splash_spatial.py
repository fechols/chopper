"""Deterministic spatial refinement for fixed-palette multicolor bitmaps.

The loss measures colors after mixing in linear light, then converting to Oklab.
Only affected filter neighborhoods are evaluated for each proposed move. This
module has no knowledge of image files, binary packing, or the gameplay renderer.
"""
from __future__ import annotations

import numpy as np


LOSS_WEIGHTS = (0.25, 0.5, 0.25)
FILTERS = (((0.6, 1.0), (1, 2)), ((1.2, 2.0), (2, 4)))
TOLERANCE = 1e-12
ZERO_EPSILON = 1e-14
PIXEL_SWEEPS = (4, 2)
SETTINGS = {
    "objective": "squared Oklab after filtering linear RGB",
    "loss_weights": dict(zip(("raw", "fine", "coarse"), LOSS_WEIGHTS)),
    "raw_normalization": "source edge-weight sum",
    "spatial_normalization": "pixel count",
    "filters": [{"sigma_xy": list(sigma), "radius_xy": list(radius)}
                for sigma, radius in FILTERS],
    "boundary": "truncate and renormalize each Gaussian at image edges",
    "pixel_sweeps_before_palette": PIXEL_SWEEPS[0],
    "palette_sweeps": 1,
    "pixel_sweeps_after_palette": PIXEL_SWEEPS[1],
    "early_stop": "pixel stage stops after a sweep with no accepted move",
    "minimum_improvement": TOLERANCE,
    "linear_rgb_zero_epsilon": ZERO_EPSILON,
    "moves": ["legal single-pixel recolor", "legal right/down pixel swap",
              "replace all occurrences of one local non-background color"],
    "tie_break": "first candidate: ascending color index, right swap, down swap",
    "scan": "alternating forward/reverse raster; row-major palette cells",
}


def _clean_linear(rgb):
    # Subtracting a color's last contribution can leave roundoff near black.
    # Oklab's cube root amplifies it enough to affect the acceptance threshold.
    # Every nonzero filtered C64 channel is much larger than this epsilon.
    np.clip(rgb, 0.0, 1.0, out=rgb)
    rgb[rgb < ZERO_EPSILON] = 0.0
    return rgb


def _axis_weights(length: int, sigma: float, radius: int) -> np.ndarray:
    if sigma <= 0 or not np.isfinite(sigma) or radius < 0 or int(radius) != radius:
        raise ValueError("Gaussian sigma must be positive and radius a nonnegative integer")
    distance = np.arange(length)[:, None] - np.arange(length)[None, :]
    weights = np.exp(-0.5 * (distance / sigma) ** 2)
    weights[np.abs(distance) > radius] = 0.0
    return weights / weights.sum(axis=1, keepdims=True)


class _Filter:
    def __init__(self, shape, sigma, radius):
        self.height, self.width = shape
        self.rx, self.ry = radius
        self.wx = _axis_weights(self.width, sigma[0], self.rx)
        self.wy = _axis_weights(self.height, sigma[1], self.ry)

    def apply(self, rgb):
        # Separable finite convolution; boundary normalization belongs to the
        # receiving pixel, not to the source pixel that contributes the color.
        vertical = np.zeros_like(rgb, dtype=np.float64)
        for dy in range(-self.ry, self.ry + 1):
            start, stop = max(0, -dy), min(self.height, self.height - dy)
            if start < stop:
                rows = np.arange(start, stop)
                vertical[start:stop] += (rgb[start + dy:stop + dy]
                                         * self.wy[rows, rows + dy, None, None])
        result = np.zeros_like(rgb, dtype=np.float64)
        for dx in range(-self.rx, self.rx + 1):
            start, stop = max(0, -dx), min(self.width, self.width - dx)
            if start < stop:
                columns = np.arange(start, stop)
                result[:, start:stop] += (vertical[:, start + dx:stop + dx]
                                          * self.wx[columns, columns + dx][None, :, None])
        return _clean_linear(result)

    def influence(self, positions):
        top = max(0, int(positions[:, 0].min()) - self.ry)
        bottom = min(self.height, int(positions[:, 0].max()) + self.ry + 1)
        left = max(0, int(positions[:, 1].min()) - self.rx)
        right = min(self.width, int(positions[:, 1].max()) + self.rx + 1)
        ys = np.repeat(np.arange(top, bottom), right - left)
        xs = np.tile(np.arange(left, right), bottom - top)
        coefficients = (self.wy[ys[:, None], positions[None, :, 0]]
                        * self.wx[xs[:, None], positions[None, :, 1]])
        return ys * self.width + xs, coefficients


def filter_linear(rgb: np.ndarray, sigma: tuple, radius: tuple) -> np.ndarray:
    """Filter RGB using Gaussian parameters in (x, y) order, without wrapping."""
    rgb = np.asarray(rgb, dtype=np.float64)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) == 0:
        raise ValueError("Expected a nonempty RGB image")
    return _Filter(rgb.shape[:2], sigma, radius).apply(rgb)


def _indices(value, shape):
    value = np.asarray(value)
    if (value.shape != shape or not np.issubdtype(value.dtype, np.integer)
            or np.any(value < 0) or np.any(value >= 16)):
        raise ValueError("Expected hardware color indices in [0, 15]")
    return value.astype(np.uint8, copy=False)


class SpatialObjective:
    """Immutable source data and full-image reference evaluation."""
    def __init__(self, linear, linear_palette, lab_palette, metric, edge_weights, to_oklab):
        linear = np.asarray(linear, dtype=np.float64)
        if (linear.ndim != 3 or linear.shape[2] != 3 or min(linear.shape[:2]) == 0
                or not np.all(np.isfinite(linear)) or np.any(linear < 0) or np.any(linear > 1)):
            raise ValueError("Source must be a nonempty finite linear RGB image in [0, 1]")
        self.shape = linear.shape[:2]
        self.size = int(np.prod(self.shape))
        self.linear_palette = np.asarray(linear_palette, dtype=np.float64)
        lab_palette = np.asarray(lab_palette, dtype=np.float64)
        self.metric = np.asarray(metric, dtype=np.float64)
        if (self.linear_palette.shape != (16, 3) or lab_palette.shape != (16, 3)
                or self.metric.shape != (3,) or not np.all(np.isfinite(self.metric))
                or np.any(self.metric <= 0) or not np.all(np.isfinite(self.linear_palette))
                or np.any(self.linear_palette < 0) or np.any(self.linear_palette > 1)
                or not np.all(np.isfinite(lab_palette))):
            raise ValueError("Expected 16 RGB/Lab palette entries and three positive metric weights")
        self.to_oklab = to_oklab
        lab = to_oklab(linear)
        weights = np.asarray(edge_weights(lab) if callable(edge_weights) else edge_weights,
                             dtype=np.float64)
        if (weights.shape != self.shape or not np.all(np.isfinite(weights))
                or np.any(weights < 0) or weights.sum() <= 0):
            raise ValueError("Expected nonnegative source edge weights with a positive sum")
        self.raw_cost = (np.sum((lab.reshape(-1, 1, 3) - lab_palette) ** 2
                                * self.metric, axis=2)
                         * (weights.reshape(-1, 1) / weights.sum()))
        self.filters = [_Filter(self.shape, sigma, radius) for sigma, radius in FILTERS]
        self.targets = [to_oklab(kernel.apply(linear)).reshape(-1, 3) for kernel in self.filters]

    def score(self, indices) -> dict:
        indices = _indices(indices, self.shape)
        raw = float(self.raw_cost[np.arange(self.size), indices.reshape(-1)].sum())
        rgb = self.linear_palette[indices]
        errors = [float(np.mean(np.sum((self.to_oklab(kernel.apply(rgb)).reshape(-1, 3)
                                        - target) ** 2 * self.metric, axis=1)))
                  for kernel, target in zip(self.filters, self.targets)]
        return _score_dict(raw, *errors)


def _score_dict(raw, fine, coarse):
    return {"raw": raw, "fine": fine, "coarse": coarse,
            "total": LOSS_WEIGHTS[0] * raw + LOSS_WEIGHTS[1] * fine + LOSS_WEIGHTS[2] * coarse}


class SpatialState:
    """Candidate evaluation against cached filtered colors and per-pixel losses.

    Positions are unique (y, x) pairs. Each candidate is a complete replacement
    for those positions. The caller supplies hardware-legal moves; this low-level
    evaluator deliberately also supports arbitrary moves for independent tests.
    """
    def __init__(self, objective: SpatialObjective, indices, background=6):
        if not 0 <= background < 16:
            raise ValueError("Invalid hardware background")
        self.objective = objective
        self.indices = _indices(indices, objective.shape).copy()
        self.background = background
        rgb = objective.linear_palette[self.indices]
        self.filtered = [kernel.apply(rgb).reshape(-1, 3) for kernel in objective.filters]
        self.losses = [np.sum((objective.to_oklab(rgb) - target) ** 2 * objective.metric, axis=1)
                       for rgb, target in zip(self.filtered, objective.targets)]

    @property
    def score(self):
        objective = self.objective
        raw = float(objective.raw_cost[np.arange(objective.size), self.indices.reshape(-1)].sum())
        return _score_dict(raw, *(float(loss.sum() / objective.size) for loss in self.losses))

    def _validate_moves(self, positions, candidates):
        positions = np.asarray(positions)
        candidates = np.asarray(candidates)
        if (positions.ndim != 2 or positions.shape[1] != 2 or len(positions) == 0
                or not np.issubdtype(positions.dtype, np.integer)
                or np.any(positions < 0) or np.any(positions >= self.objective.shape)
                or len(np.unique(positions, axis=0)) != len(positions)):
            raise ValueError("Expected distinct in-bounds (y, x) positions")
        if (candidates.ndim != 2 or candidates.shape[1] != len(positions)
                or len(candidates) == 0 or not np.issubdtype(candidates.dtype, np.integer)
                or np.any(candidates < 0) or np.any(candidates > 15)):
            raise ValueError("Expected a nonempty candidate matrix of hardware colors")
        return positions.astype(np.intp, copy=False), candidates.astype(np.uint8, copy=False)

    def _evaluate(self, positions, candidates):
        objective = self.objective
        flat = positions[:, 0] * objective.shape[1] + positions[:, 1]
        old = self.indices.reshape(-1)[flat]
        delta = objective.linear_palette[candidates] - objective.linear_palette[old]
        scores = LOSS_WEIGHTS[0] * np.sum(objective.raw_cost[flat[None, :], candidates]
                                        - objective.raw_cost[flat, old], axis=1)
        patches = []
        for scale, (kernel, target) in enumerate(zip(objective.filters, objective.targets)):
            affected, influence = kernel.influence(positions)
            rgb = (self.filtered[scale][affected][None, :, :]
                   + np.einsum("pn,knc->kpc", influence, delta, optimize=False))
            _clean_linear(rgb)
            losses = np.sum((objective.to_oklab(rgb) - target[affected]) ** 2
                            * objective.metric, axis=2)
            scores += (LOSS_WEIGHTS[scale + 1] / objective.size
                       * np.sum(losses - self.losses[scale][affected], axis=1))
            patches.append((affected, rgb, losses))
        return scores, patches

    def delta_scores(self, positions, candidates):
        """Return exact local score changes without accepting any candidate."""
        positions, candidates = self._validate_moves(positions, candidates)
        return self._evaluate(positions, candidates)[0]

    def _try(self, positions, candidates):
        deltas, patches = self._evaluate(positions, candidates)
        best = int(np.argmin(deltas))
        if deltas[best] >= -TOLERANCE:
            return False
        self.indices[positions[:, 0], positions[:, 1]] = candidates[best]
        for scale, (affected, rgb, losses) in enumerate(patches):
            self.filtered[scale][affected] = rgb[best]
            self.losses[scale][affected] = losses[best]
        return True

    def try_changes(self, positions, candidates) -> bool:
        positions, candidates = self._validate_moves(positions, candidates)
        return self._try(positions, candidates)


def refine(objective: SpatialObjective, indices, background=6, progress=None) -> np.ndarray:
    """Refine a legal bitmap with a fixed, bounded sequence of greedy sweeps."""
    height, width = objective.shape
    if height % 8 or width % 4:
        raise ValueError("Refinement requires complete 4x8 multicolor cells")
    state = SpatialState(objective, indices, background)
    image = state.indices
    cells_across = width // 4
    counts = np.zeros((height // 8 * cells_across, 16), dtype=np.int16)
    cell_ids = (np.arange(height)[:, None] // 8 * cells_across
                + np.arange(width)[None, :] // 4)
    np.add.at(counts, (cell_ids.reshape(-1), image.reshape(-1)), 1)
    nonbackground = np.arange(16) != background
    if np.any(np.count_nonzero(counts[:, nonbackground], axis=1) > 3):
        raise ValueError("Seed exceeds three local non-background colors")

    def accept(positions, candidates):
        old = image[positions[:, 0], positions[:, 1]].copy()
        if not state._try(positions, candidates):
            return False
        for (y, x), before in zip(positions, old):
            after = image[y, x]
            if before != after:
                cell = cell_ids[y, x]
                counts[cell, before] -= 1
                counts[cell, after] += 1
        return True

    def swap_legal(cell, old, new):
        used = int(np.count_nonzero(counts[cell, nonbackground]))
        used -= int(old != background and counts[cell, old] == 1)
        used += int(new != background and counts[cell, new] == 0)
        return used <= 3

    sweep_number = 0

    def pixel_stage(limit):
        nonlocal sweep_number
        for _ in range(limit):
            reverse = bool(sweep_number % 2)
            changed = 0
            order = range(height * width - 1, -1, -1) if reverse else range(height * width)
            for flat in order:
                y, x = divmod(flat, width)
                old = int(image[y, x])
                cell = int(cell_ids[y, x])
                remaining = counts[cell].copy()
                remaining[old] -= 1
                colors = (np.arange(16) if np.count_nonzero(remaining[nonbackground]) <= 2
                          else np.flatnonzero((remaining > 0) | ~nonbackground))
                colors = colors[colors != old]
                positions = [(y, x)]
                swaps = []
                for ny, nx in ((y, x + 1), (y + 1, x)):
                    if ny >= height or nx >= width:
                        continue
                    other = int(image[ny, nx])
                    other_cell = int(cell_ids[ny, nx])
                    if old == other:
                        continue
                    if cell == other_cell or (swap_legal(cell, old, other)
                                               and swap_legal(other_cell, other, old)):
                        swaps.append(len(positions))
                        positions.append((ny, nx))
                positions = np.asarray(positions, dtype=np.intp)
                current = image[positions[:, 0], positions[:, 1]]
                candidates = np.tile(current, (len(colors) + len(swaps), 1))
                candidates[:len(colors), 0] = colors
                for row, slot in enumerate(swaps, len(colors)):
                    candidates[row, 0], candidates[row, slot] = current[slot], current[0]
                if len(candidates) and accept(positions, candidates):
                    changed += 1
            sweep_number += 1
            if progress:
                progress(f"Spatial pixel sweep {sweep_number}: {changed} moves, score {state.score['total']:.10f}")
            if not changed:
                break

    pixel_stage(PIXEL_SWEEPS[0])
    changed = 0
    for cy in range(height // 8):
        for cx in range(cells_across):
            cell = cy * cells_across + cx
            # Take a stable snapshot of the current colors. A replacement into
            # a later color may be considered again; all moves still decrease loss.
            colors = np.flatnonzero((counts[cell] > 0) & nonbackground)
            for color in colors:
                positions = np.argwhere(image[cy * 8:cy * 8 + 8, cx * 4:cx * 4 + 4] == color)
                if len(positions) == 0:
                    continue
                positions += (cy * 8, cx * 4)
                alternatives = np.arange(16, dtype=np.uint8)
                alternatives = alternatives[alternatives != color]
                candidates = np.repeat(alternatives[:, None], len(positions), axis=1)
                changed += int(accept(positions, candidates))
    if progress:
        progress(f"Spatial palette sweep: {changed} moves, score {state.score['total']:.10f}")
    pixel_stage(PIXEL_SWEEPS[1])
    return image
