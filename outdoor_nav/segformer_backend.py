"""Offline Hugging Face SegFormer adapter returning unmodified model class IDs."""
import json
import math
from pathlib import Path
import numpy as np


class SegformerBackend:
    def __init__(self, model, processor, device):
        import torch
        self.torch = torch
        self.device = torch.device(device)
        self.model = model.to(self.device).eval()
        self.processor = processor
        if not 1 <= model.config.num_labels <= 65535:
            raise ValueError('SegFormer num_labels must be in [1, 65535]')

    def predict(self, rgb):
        if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or not rgb.size:
            raise ValueError('SegFormer expects nonempty HxWx3 uint8 RGB')
        with self.torch.inference_mode():
            inputs = self.processor(images=rgb, return_tensors='pt')
            inputs = {key: value.to(self.device) for key, value in inputs.items()}
            logits = self.model(**inputs).logits
            if logits.ndim != 4 or logits.shape[0] != 1 or logits.shape[1] != self.model.config.num_labels:
                raise ValueError('Unexpected SegFormer logits shape')
            logits = self.torch.nn.functional.interpolate(
                logits.float(), size=rgb.shape[:2], mode='bilinear', align_corners=False)
            confidence, labels = logits.softmax(dim=1).max(dim=1)
        return (labels[0].cpu().numpy().astype(np.uint16),
                confidence[0].cpu().numpy().astype(np.float32))


def explicit_preprocessing(settings):
    if not isinstance(settings, dict):
        raise ValueError('SegFormer options.preprocessing must be a mapping')
    flags = ('do_resize', 'do_rescale', 'do_normalize')
    allowed = set(flags) | {'size', 'resample', 'rescale_factor', 'image_mean', 'image_std'}
    if set(settings) - allowed or any(type(settings.get(key)) is not bool for key in flags):
        raise ValueError('SegFormer explicit preprocessing requires boolean do_resize, do_rescale and '
                         'do_normalize; see README for supported fields')
    if settings['do_resize']:
        size = settings.get('size')
        resample = settings.get('resample')
        if (not isinstance(size, dict) or set(size) != {'height', 'width'}
                or any(type(n) is not int or n <= 0 for n in size.values())
                or type(resample) is not int or not 0 <= resample <= 5):
            raise ValueError('SegFormer resize needs a positive height/width and Pillow resample (0–5)')
    if settings['do_rescale']:
        factor = settings.get('rescale_factor')
        if type(factor) not in (int, float) or not math.isfinite(factor) or factor <= 0:
            raise ValueError('SegFormer rescale_factor must be finite and positive')
    if settings['do_normalize']:
        for key in ('image_mean', 'image_std'):
            values = settings.get(key)
            if (not isinstance(values, list) or len(values) != 3
                    or any(type(value) not in (int, float) or not math.isfinite(value)
                           or (key == 'image_std' and value <= 0) for value in values)):
                raise ValueError(f'SegFormer {key} needs three finite RGB values (positive for std)')
    return settings


def create_backend(*, weights, device, options):
    if not isinstance(options, dict) or set(options) - {'preprocessing'}:
        raise ValueError('SegFormer options only supports preprocessing; see README')
    path = Path(weights)
    if not path.is_dir():
        raise ValueError('SegFormer weights must be a local save_pretrained model directory; see README')
    if not (path / 'config.json').is_file():
        raise ValueError('SegFormer model directory requires config.json; see README')
    explicit = 'preprocessing' in options
    if explicit:
        preprocessing = explicit_preprocessing(options['preprocessing'])
    else:
        processor_config = path / 'preprocessor_config.json'
        if not processor_config.is_file():
            raise ValueError('Set semantic_segmentation.options.preprocessing or supply preprocessor_config.json '
                             'with the training settings; see README')
        preprocessing = json.loads(processor_config.read_text(encoding='utf-8'))
        # Cropping/padding needs an inverse mapping before projecting labels into depth.
        if not isinstance(preprocessing, dict) or preprocessing.get('do_center_crop') or preprocessing.get('do_pad'):
            raise ValueError('SegFormer adapter does not support crop/pad preprocessing; pixel alignment must be preserved')
    try:
        import torch
        from transformers import SegformerForSemanticSegmentation
        try:
            from transformers import SegformerImageProcessorPil as Processor
        except ImportError:
            from transformers import SegformerImageProcessor as Processor
    except ImportError as exc:
        raise RuntimeError('SegFormer requires compatible torch and transformers in the worker environment; see README [segformer] installation') from exc
    try:
        processor = (Processor(**preprocessing) if explicit else
                     Processor.from_pretrained(str(path), local_files_only=True))
        model = SegformerForSemanticSegmentation.from_pretrained(str(path), local_files_only=True)
    except (OSError, ValueError) as exc:
        raise ValueError('Cannot load local SegFormer model or preprocessing; see README. ' + str(exc)) from exc
    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    return SegformerBackend(model, processor, device)
