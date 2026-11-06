"""Eye-to-Hand: G_i M = X C_i. X=T_B_C, M=T_G_T, G_i=T_B_G.

OpenCV receives inverse(G_i) as its first transform and C_i=T_C_T as second.
Outputs require geometric excitation and independent fixed-mount residual checks.
"""
from itertools import combinations
import math
from pathlib import Path
import cv2
import numpy as np
from .transforms import inverse, transform, validate_transform, mean_transform, pose_error


class CalibrationError(ValueError):
    pass


def excitation(poses,min_rotation_deg=3.):
    rotations=[]
    translations=[]
    for a,b in combinations(poses,2):
        relative=inverse(a)@b
        rotations.append(cv2.Rodrigues(relative[:3,:3])[0].reshape(3))
        translations.append(relative[:3,:3]-np.eye(3))
    singular=np.linalg.svd(np.array(rotations),compute_uv=False)
    matrix=np.concatenate(translations,axis=0)
    condition=np.linalg.cond(matrix)
    if (len(singular)<2 or singular[1]<1e-3*max(singular[0],1e-12) or
            max(np.linalg.norm(v) for v in rotations)<np.radians(min_rotation_deg) or
            np.linalg.matrix_rank(matrix)<3 or not np.isfinite(condition) or condition>1e6):
        raise CalibrationError('DEGENERATE_MOTION: independent rotation axes/translation observability insufficient')
    return {'rotation_singular_values':singular.tolist(),'translation_condition':float(condition)}


def _opencv(gripper,camera):
    inverted=[inverse(g) for g in gripper]
    r,t=cv2.calibrateHandEye([g[:3,:3] for g in inverted],[g[:3,3] for g in inverted],
                    [c[:3,:3] for c in camera],[c[:3,3] for c in camera],method=cv2.CALIB_HAND_EYE_PARK)
    return transform(r,t)


def solve_eye_to_hand(T_base_flange,T_camera_board,translation_threshold_mm=10.,rotation_threshold_deg=3.,
                       min_inlier_ratio=.65,robust_trials=100,seed=42):
    g=[validate_transform(v) for v in T_base_flange]
    c=[validate_transform(v) for v in T_camera_board]
    n=len(g)
    if n!=len(c) or n<3:
        raise CalibrationError('At least three matched pose pairs required')
    if min(translation_threshold_mm,rotation_threshold_deg)<=0 or not 0<min_inlier_ratio<=1:
        raise ValueError('Invalid residual/consensus thresholds')
    excitation(g)
    rng=np.random.default_rng(seed)
    candidates=[np.arange(n)]
    subset=min(n,max(3,min(6,n//2)))
    candidates += [rng.choice(n,subset,replace=False) for _ in range(robust_trials)]
    best=None
    for indices in candidates:
        try:
            excitation([g[i] for i in indices])
            x=_opencv([g[i] for i in indices],[c[i] for i in indices])
            mount=mean_transform([inverse(g[i])@x@c[i] for i in indices])
            errors=[pose_error(gv@mount,x@cv) for gv,cv in zip(g,c)]
            inliers=[i for i,e in enumerate(errors) if e['translation_mm']<=translation_threshold_mm and e['rotation_deg']<=rotation_threshold_deg]
            score=(len(inliers),-sum(errors[i]['translation_mm']/translation_threshold_mm+errors[i]['rotation_deg']/rotation_threshold_deg for i in inliers))
            if best is None or score>best[0]:
                best=(score,inliers)
        except (ValueError,cv2.error,np.linalg.LinAlgError):
            continue
    if best is None or len(best[1])<max(3,math.ceil(n*min_inlier_ratio)):
        raise CalibrationError('NO_CONSISTENT_SOLUTION: direction, synchronization or outliers invalid')
    indices=best[1]
    diagnostics=excitation([g[i] for i in indices])
    x=_opencv([g[i] for i in indices],[c[i] for i in indices])
    mount=mean_transform([inverse(g[i])@x@c[i] for i in indices])
    errors=[pose_error(gv@mount,x@cv) for gv,cv in zip(g,c)]
    if any(errors[i]['translation_mm']>translation_threshold_mm or errors[i]['rotation_deg']>rotation_threshold_deg for i in indices):
        raise CalibrationError('REFINED_SOLUTION_INCONSISTENT')
    return {'T_base_camera':x.tolist(),'T_flange_board':mount.tolist(),
            'rotation_matrix':x[:3,:3].tolist(),'translation_m':x[:3,3].tolist(),
            'coordinate_convention':'T_A_B maps B to A; G_i M = X C_i; B base, G flange, C RGB optical, T board',
            'algorithm':'OpenCV PARK; inverse(T_B_G), T_C_T; deterministic consensus then refit',
            'sample_count':n,'valid_sample_count':len(indices),'valid_sample_ratio':len(indices)/n,
            'inlier_indices':[int(i) for i in indices],
            'residuals':[dict(sample_index=i,inlier=i in indices,**e) for i,e in enumerate(errors)],
            'max_inlier_translation_residual_mm':max(errors[i]['translation_mm'] for i in indices),
            'max_inlier_rotation_residual_deg':max(errors[i]['rotation_deg'] for i in indices),
            'excitation':diagnostics,'validity':True,
            'input_samples':[{'sample_index':i,'T_base_flange':gv.tolist(),'T_camera_board':cv.tolist()} for i,(gv,cv) in enumerate(zip(g,c))]}

