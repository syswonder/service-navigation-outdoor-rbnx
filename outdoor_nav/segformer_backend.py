"""Offline Hugging Face SegFormer adapter returning unmodified model class IDs."""
import json
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


def create_backend(*, weights, device, options):
    if not isinstance(options, dict) or options:
        raise ValueError('SegFormer options must be empty; save preprocessing settings with the weights')
    path = Path(weights)
    if not path.is_dir():
        raise ValueError('SegFormer weights must be a local save_pretrained model directory; see README')
    for name in ('config.json', 'preprocessor_config.json'):
        if not (path / name).is_file():
            raise ValueError('SegFormer model directory requires '+name+'; see README')
    preprocessing = json.loads((path / 'preprocessor_config.json').read_text(encoding='utf-8'))
    # Resizing preserves the whole view; cropping/padding need an explicit inverse mapping.
    if preprocessing.get('do_center_crop') or preprocessing.get('do_pad'):
        raise ValueError('SegFormer adapter does not support crop/pad preprocessing; pixel alignment must be preserved')
    try:
        import torch
        from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
    except ImportError as exc:
        raise RuntimeError('SegFormer requires compatible torch and transformers in the worker environment; see README [segformer] installation') from exc
    try:
        processor = SegformerImageProcessor.from_pretrained(str(path), local_files_only=True)
        model = SegformerForSemanticSegmentation.from_pretrained(str(path), local_files_only=True)
    except (OSError, ValueError) as exc:
        raise ValueError('Cannot load local SegFormer model weights/configuration. Supply a complete compatible save_pretrained directory; see README. ' + str(exc)) from exc
    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    return SegformerBackend(model, processor, device)
