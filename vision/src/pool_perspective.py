"""Four pool corners -> rectangular field coordinates. No fish/control logic."""
import json
from pathlib import Path

import cv2
import numpy as np


def transform_points(matrix,points):
    if matrix is None:raise ValueError('请先完成场地四点标定')
    points=np.asarray(points,dtype=np.float64)
    if points.ndim!=2 or points.shape[1]!=2 or not np.isfinite(points).all():
        raise ValueError('采样点必须为有限的 [x, y] 坐标')
    homogeneous=np.column_stack((points,np.ones(len(points))))@np.asarray(matrix,dtype=np.float64).T
    if not np.isfinite(homogeneous).all() or np.any(abs(homogeneous[:,2])<1e-9):
        raise ValueError('采样点位于无效透视范围')
    return homogeneous[:,:2]/homogeneous[:,2,None]


def image_heading(matrix,body,head):
    """Transform both endpoints before measuring CCW heading in field metres."""
    world=transform_points(matrix,[body,head]);direction=world[1]-world[0]
    if np.linalg.norm(direction)<.02:raise ValueError('鱼头方向映射距离过短，请重新点击')
    return float(np.arctan2(-direction[1],direction[0]))


def heading_arrow(inverse,origin_world,heading_rad,length_m=.22):
    """Project both ends of the SAME world vector into the original image."""
    origin=np.asarray(origin_world,dtype=np.float64)
    tip=origin+length_m*np.array([np.cos(heading_rad),-np.sin(heading_rad)])
    return transform_points(inverse,[origin,tip])


class PoolPerspective:
    def __init__(self,width_m=4.,height_m=2.):
        self.width_m=float(width_m);self.height_m=float(height_m)
        self.matrix=None;self.points=None;self.frame_shape=None

    def calibrate(self,points,frame_shape):
        points=np.asarray(points,dtype=np.float32)
        if points.shape!=(4,2) or not np.isfinite(points).all():
            raise ValueError('请依次选择左上、右上、右下、左下四个点')
        height,width=frame_shape[:2]
        if not ((points[:,0]>=0)&(points[:,0]<width)&(points[:,1]>=0)&(points[:,1]<height)).all():
            raise ValueError('四个点必须位于原始图像内')
        if not cv2.isContourConvex(points) or cv2.contourArea(points)<width*height*.05:
            raise ValueError('四点顺序错误、四边形交叉或面积过小，请重新选择')
        if (points[0,1]+points[1,1]>=points[2,1]+points[3,1] or
                points[0,0]+points[3,0]>=points[1,0]+points[2,0]):
            raise ValueError('请按左上→右上→右下→左下点击')
        if not all(np.isfinite(v) and .05<=v<=50 for v in (self.width_m,self.height_m)):
            raise ValueError('场地长宽必须在 0.05～50 米内')
        target=np.float32([[0,0],[self.width_m,0],[self.width_m,self.height_m],[0,self.height_m]])
        matrix=cv2.getPerspectiveTransform(points,target)
        if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix))<1e-12:
            raise ValueError('无法计算有效的透视变换矩阵')
        self.matrix=matrix;self.points=points;self.frame_shape=(int(height),int(width))
        return self

    def transform(self,points):
        """Apply the cached matrix to sampling points; no full-frame warp."""
        return transform_points(self.matrix,points)

    def rectify(self,frame,max_side=1000):
        """One explicitly requested test image. Never call this in a frame loop."""
        if self.matrix is None:raise ValueError('请先完成场地四点标定')
        if tuple(frame.shape[:2])!=self.frame_shape:raise ValueError('图像分辨率已改变，请重新标定')
        scale=max_side/max(self.width_m,self.height_m)
        width=max(2,round(self.width_m*scale));height=max(2,round(self.height_m*scale))
        pixels=np.diag([(width-1)/self.width_m,(height-1)/self.height_m,1.])
        return cv2.warpPerspective(frame,pixels@self.matrix,(width,height))

    def describe(self,source):
        return {'version':1,'camera_source':str(source),'frame_width':self.frame_shape[1],
                'frame_height':self.frame_shape[0],'dimension_source':'configured',
                'physical_width_m':self.width_m,'physical_height_m':self.height_m,
                'points_tl_tr_br_bl':self.points.tolist(),'image_to_field':self.matrix.tolist(),
                'field_to_image':np.linalg.inv(self.matrix).tolist(),
                'field_axes':'origin top-left; x right; y down; metres',
                'mapped_corners_m':self.transform(self.points).tolist()}

    def save(self,path,source):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(json.dumps(self.describe(source),ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
        temporary.replace(path)

    def load(self,path,source,frame_shape):
        try:
            value=json.loads(Path(path).read_text(encoding='utf-8'))
            if (value.get('version')!=1 or value.get('camera_source')!=str(source) or
                    (value.get('frame_height'),value.get('frame_width'))!=tuple(frame_shape[:2])):return False
            updated=PoolPerspective(value['physical_width_m'],value['physical_height_m'])
            updated.calibrate(value['points_tl_tr_br_bl'],frame_shape)
            self.__dict__.update(updated.__dict__)
            return True
        except (OSError,KeyError,TypeError,ValueError,cv2.error):return False
