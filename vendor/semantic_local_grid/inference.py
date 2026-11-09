"""PIDNet-M official architecture; strict inference-weight validation."""
import os
from pathlib import Path
import numpy as np
import cv2

def traversability_mask(classes, confidence, threshold=.7, allowed_train_ids=(0,1)):
    """Cityscapes train IDs: road=0, sidewalk=1; binary output contract unchanged."""
    allowed=tuple(allowed_train_ids)
    if not allowed or any(isinstance(x,bool) or not isinstance(x,(int,np.integer)) or x not in (0,1) for x in allowed):
        raise ValueError('Only road/sidewalk train IDs may be traversable')
    if not np.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ValueError('Invalid confidence threshold')
    classes,confidence=np.asarray(classes),np.asarray(confidence)
    if classes.shape != confidence.shape: raise ValueError('Class/confidence dimensions differ')
    mask=np.isin(classes,allowed).astype(np.uint8)
    mask[(confidence<threshold)|~np.isfinite(confidence)]=255
    return mask

class Segmenter:
    def __init__(self,weights='',device='auto',model='pidnet-m',confidence=0.7,threads=4,traversable_train_ids=(0,1)):
        import torch
        from .pidnet_vendor.pidnet import get_pred_model
        if model not in ('pidnet-s','pidnet-m','pidnet-l'): raise ValueError('Invalid PIDNet architecture')
        path=Path(weights or os.environ.get('PIDNET_WEIGHTS','')).expanduser()
        if not path.is_file(): raise FileNotFoundError('Set pidnet_weights or PIDNET_WEIGHTS to a checkpoint')
        torch.set_num_threads(threads)
        self.torch=torch
        self.device=torch.device(('cuda' if torch.cuda.is_available() else 'cpu') if device=='auto' else device)
        self.model=get_pred_model(model,19)
        state=torch.load(path,map_location='cpu',weights_only=True)
        if 'state_dict' in state: state=state['state_dict']
        normalized={}
        for key,value in state.items():
            while key.startswith(('model.','module.')):
                key=key.split('.',1)[1]
            normalized[key]=value
        expected=self.model.state_dict()
        missing=[k for k in expected if k not in normalized or normalized[k].shape!=expected[k].shape]
        if missing: raise ValueError('Incomplete/incompatible inference weights: '+str(missing[:10]))
        self.model.load_state_dict({k:normalized[k] for k in expected},strict=True)
        self.model.to(self.device).eval()
        self.confidence=confidence
        self.traversable_train_ids=tuple(traversable_train_ids)
        traversability_mask(np.array([0]),np.array([1.]),confidence,self.traversable_train_ids)
        self.metadata={'device':str(self.device),'architecture':model,'loaded_tensors':len(expected),
                       'checkpoint_tensors':len(state),'classes':19,'road_train_id':0,
                       'traversable_train_ids':list(self.traversable_train_ids)}
    def predict(self,rgb):
        torch=self.torch
        h,w=rgb.shape[:2]
        # Pad to 64 multiples; never stretch the image or change pixel coordinates.
        ph,pw=(-h)%64,(-w)%64
        padded=cv2.copyMakeBorder(rgb,0,ph,0,pw,cv2.BORDER_REPLICATE)
        x=(padded.astype(np.float32)/255.0-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
        x=torch.from_numpy(x.transpose(2,0,1).copy()).unsqueeze(0).to(self.device)
        with torch.inference_mode():
            logits=self.model(x)
            logits=torch.nn.functional.interpolate(logits,size=padded.shape[:2],mode='bilinear',align_corners=True)
            probabilities=logits.softmax(1)
            conf,classes=probabilities.max(1)
            conf=conf[0,:h,:w].cpu().numpy()
            classes=classes[0,:h,:w].cpu().numpy()
        mask=traversability_mask(classes,conf,self.confidence,self.traversable_train_ids)
        return mask,conf

def mock_mask(rgb):
    h,w=rgb.shape[:2]
    out=np.zeros((h,w),np.uint8)
    polygon=np.array([[int(.45*w),int(.55*h)],[int(.55*w),int(.55*h)],
                      [int(.85*w),h-1],[int(.15*w),h-1]])
    cv2.fillPoly(out,[polygon],1)
    return out

def overlay(rgb,mask):
    out=rgb.copy()
    road=mask==1
    out[road]=(out[road].astype(float)*.55+np.array([0,255,0])*.45).astype(np.uint8)
    return out
