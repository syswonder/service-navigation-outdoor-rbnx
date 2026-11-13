"""Replaceable segmentation backends; model class IDs belong to configuration."""
import importlib
import math
from pathlib import Path
import re

UNKNOWN = 65535


def validate_segmentation(raw, base, camera_enabled):
    defaults = dict(enabled=False, backend='', weights='', device='auto', options={},
                    confidence_threshold=.7, classes={}, road_classes=[])
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
    if not isinstance(cfg['classes'], dict):
        raise ValueError('semantic_segmentation.classes must map model IDs to class names')
    classes = {}
    for key, name in cfg['classes'].items():
        if type(key) is not int and not (isinstance(key, str) and re.fullmatch(r'[0-9]+', key)):
            raise ValueError('Segmentation class IDs must be integers in [0, 65534]')
        index = int(key)
        if not 0 <= index < UNKNOWN or str(index) in classes:
            raise ValueError('Segmentation class IDs must be unique and in [0, 65534]')
        if not isinstance(name, str) or not name.strip() or name != name.strip():
            raise ValueError('Segmentation class names must be nonempty trimmed strings')
        classes[str(index)] = name
    cfg['classes'] = classes
    roads = cfg['road_classes']
    if (not isinstance(roads, list) or any(not isinstance(v, str) for v in roads)
            or len(roads) != len(set(roads)) or set(roads) - set(classes.values())):
        raise ValueError('road_classes must be unique names declared in classes')
    if cfg['weights']:
        cfg['weights'] = str((Path(base) / Path(cfg['weights']).expanduser()).resolve())
    if cfg['enabled']:
        if not camera_enabled:
            raise ValueError('semantic_segmentation.enabled requires policy.camera_enabled')
        if not re.fullmatch(r'[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*:[A-Za-z_]\w*', cfg['backend']):
            raise ValueError('Set semantic_segmentation.backend to an importable module:factory; see README')
        if not cfg['weights'] or not (Path(cfg['weights']).is_file() or Path(cfg['weights']).is_dir()):
            raise ValueError('Semantic segmentation requires model weights: set semantic_segmentation.weights to an existing file or directory. The adapter validates its format. No weights are bundled or downloaded; see README.')
        if not classes:
            raise ValueError('Set semantic_segmentation.classes using the model class table; see README')
    return cfg


def create_backend(cfg):
    if not cfg['enabled']:
        raise ValueError('Semantic segmentation is disabled')
    if not cfg['weights'] or not (Path(cfg['weights']).is_file() or Path(cfg['weights']).is_dir()):
        raise ValueError('Missing semantic_segmentation.weights; supply the model weights described in README')
    module, name = cfg['backend'].split(':')
    backend = getattr(importlib.import_module(module), name)(
        weights=cfg['weights'], device=cfg['device'], options=cfg['options'])
    if not callable(getattr(backend, 'predict', None)):
        raise TypeError('Segmentation factory must return an object with predict(rgb)')
    return backend


def predict_labels(backend, rgb, cfg):
    """Return model labels (uint16), confidence and the existing road mask (uint8)."""
    import numpy as np
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or not rgb.size:
        raise ValueError('Segmentation input must be nonempty uint8 HxWx3 RGB')
    labels, confidence = backend.predict(rgb)
    labels, confidence = np.asarray(labels), np.asarray(confidence)
    if labels.shape != rgb.shape[:2] or confidence.shape != labels.shape:
        raise ValueError('Segmentation output must preserve input HxW pixel alignment')
    if not np.issubdtype(labels.dtype, np.integer) or ((labels < 0) | (labels > UNKNOWN)).any():
        raise ValueError('Segmentation labels must be integer IDs in [0, 65535]; 65535 is unknown')
    if not np.issubdtype(confidence.dtype, np.floating) or not np.isfinite(confidence).all() or ((confidence < 0) | (confidence > 1)).any():
        raise ValueError('Segmentation confidence must contain finite floating probabilities in [0, 1]')
    classes = {int(k): v for k, v in cfg['classes'].items()}
    labels = labels.astype('<u2', copy=True)
    labels[(confidence < cfg['confidence_threshold']) | ~np.isin(labels, list(classes))] = UNKNOWN
    road = np.zeros(labels.shape, np.uint8)
    road[np.isin(labels, [k for k, v in classes.items() if v in cfg['road_classes']])] = 1
    road[labels == UNKNOWN] = 255
    return labels, confidence.astype('<f4', copy=True), road
