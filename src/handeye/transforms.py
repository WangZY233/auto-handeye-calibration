"""SE(3) helpers. T_A_B maps B coordinates into A; lengths are metres."""
import cv2
import numpy as np


def validate_transform(value):
    value=np.asarray(value,dtype=float)
    if value.shape!=(4,4) or not np.isfinite(value).all() or not np.allclose(value[3],[0,0,0,1],atol=1e-9):
        raise ValueError('Invalid homogeneous transform')
    r=value[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(r),1.,atol=1e-7):
        raise ValueError('Rotation must be in SO(3)')
    return value


def transform(rotation,translation):
    value=np.eye(4)
    rotation=np.asarray(rotation,dtype=float)
    value[:3,:3]=cv2.Rodrigues(rotation.reshape(3))[0] if rotation.size==3 else rotation
    value[:3,3]=np.asarray(translation).reshape(3)
    return validate_transform(value)


def inverse(value):
    value=validate_transform(value)
    result=np.eye(4)
    result[:3,:3]=value[:3,:3].T
    result[:3,3]=-result[:3,:3]@value[:3,3]
    return result


def pose_error(reference,estimate):
    delta=inverse(reference)@validate_transform(estimate)
    r=delta[:3,:3]
    sine=np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])/2
    angle=np.arctan2(sine,np.clip((np.trace(r)-1)/2,-1,1))
    return {'translation_mm':float(np.linalg.norm(delta[:3,3])*1000),
            'rotation_deg':float(np.degrees(angle))}


def mean_transform(values):
    values=np.asarray([validate_transform(v) for v in values])
    u,_,vt=np.linalg.svd(values[:,:3,:3].sum(axis=0))
    r=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    return transform(r,values[:,:3,3].mean(axis=0))
