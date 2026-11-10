"""Model-neutral road segmentation contract and optional backend loading."""
import importlib
import math
from pathlib import Path
import re


def validate_segmentation(raw, base, camera_enabled):
    defaults = dict(enabled=False, backend='', weights='', device='auto', options={}, confidence_threshold=.7)
    if not isinstance(raw, dict) or set(raw) - defaults.keys():
        raise ValueError('Unknown semantic_segmentation fields; see README')
    cfg = {**defaults, **raw}
    if type(cfg['enabled']) is not bool or not isinstance(cfg['options'], dict):
        raise ValueError('semantic_segmentation requires boolean enabled and mapping options')
    for key in ('backend', 'weights', 'device'):
        if not isinstance(cfg[key], str):
            raise ValueError('semantic_segmentation.' + key + ' must be a string')
    threshold = cfg['confidence_threshold']
    if isinstance(threshold, bool) or not isinstance(threshold, (int, float)) or not math.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ValueError('semantic_segmentation.confidence_threshold must be in [0, 1]')
    if cfg['weights']:
        cfg['weights'] = str((base / Path(cfg['weights']).expanduser()).resolve())
    if cfg['enabled']:
        if not camera_enabled:
            raise ValueError('semantic_segmentation.enabled requires policy.camera_enabled')
        if cfg['backend'] != 'pidnet' and not re.fullmatch(r'[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*:[A-Za-z_]\w*', cfg['backend']):
            raise ValueError('Choose semantic_segmentation.backend: pidnet or importable module:factory; see README')
        if not cfg['weights'] or not Path(cfg['weights']).exists():
            raise ValueError('Semantic segmentation requires downloaded model weights: set semantic_segmentation.weights to an existing file or model directory. Weights are not bundled or downloaded automatically; see README download links.')
    return cfg


def create_backend(cfg):
    if not cfg['enabled']:
        raise ValueError('Semantic segmentation is disabled')
    if not Path(cfg['weights']).exists():
        raise ValueError('Missing semantic_segmentation.weights; download the model weights described in README')
    if cfg['backend'] == 'pidnet':
        # Only this optional adapter knows PIDNet / Cityscapes class IDs.
        if not Path(cfg['weights']).is_file():
            raise ValueError('PIDNet weights must be a .pt file')
        from semantic_local_grid.inference import Segmenter
        options = cfg['options']
        if set(options) - {'architecture', 'threads', 'traversable_train_ids'}:
            raise ValueError('Unknown PIDNet backend options')
        return Segmenter(cfg['weights'], cfg['device'], options.get('architecture', 'pidnet-m'),
                         cfg['confidence_threshold'], options.get('threads', 4),
                         options.get('traversable_train_ids', [0, 1]))
    module, name = cfg['backend'].split(':')
    backend = getattr(importlib.import_module(module), name)(
        weights=cfg['weights'], device=cfg['device'], options=cfg['options'])
    if not callable(getattr(backend, 'predict', None)):
        raise TypeError('Segmentation factory must return an object with predict(rgb)')
    return backend


def predict_mask(backend, rgb, threshold):
    """RGB uint8 HxWx3 -> mask uint8 HxW: 0 nonroad, 1 road, 255 unknown."""
    import numpy as np
    mask, confidence = backend.predict(rgb)
    mask, confidence = np.asarray(mask), np.asarray(confidence)
    if mask.shape != rgb.shape[:2] or confidence.shape != mask.shape:
        raise ValueError('Segmentation output must preserve input HxW pixel alignment')
    if mask.dtype != np.uint8 or not np.isin(mask, [0, 1, 255]).all():
        raise ValueError('Segmentation mask must be uint8 with labels 0/1/255')
    if not np.issubdtype(confidence.dtype, np.number) or not np.isfinite(confidence).all() or ((confidence < 0) | (confidence > 1)).any():
        raise ValueError('Segmentation confidence must contain finite probabilities in [0, 1]')
    mask = mask.copy()
    mask[confidence < threshold] = 255
    return mask
