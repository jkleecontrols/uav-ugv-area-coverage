"""Local-window camera rasterization, shared by planning and measurement."""
from __future__ import annotations

import numpy as np
from shapely import contains_xy


class Raster:
    def __init__(self, aoi, resolution, forward=100., cross=67.):
        self.aoi = aoi
        if resolution <= 0 or forward <= 0 or cross <= 0:
            raise ValueError('Resolution and footprint dimensions must be positive')
        x0, y0, x1, y1 = aoi.bounds
        self.x = np.arange(x0+resolution/2, x1, resolution)
        self.y = np.arange(y0+resolution/2, y1, resolution)
        self.mask = contains_xy(aoi, self.x[None, :], self.y[:, None])
        self.shape = self.mask.shape
        self.resolution, self.forward, self.cross = resolution, forward, cross
        self.size = self.mask.size
        self.valid = self.mask.ravel()
        if not self.valid.any():
            raise ValueError('AOI contains no cell centres')

    def footprint(self, pose):
        x, y, h = pose
        c, s = np.cos(h), np.sin(h)
        dx = (abs(c)*self.forward+abs(s)*self.cross)/2
        dy = (abs(s)*self.forward+abs(c)*self.cross)/2
        ix0, ix1 = np.searchsorted(self.x, [x-dx-1e-8, x+dx+1e-8])
        iy0, iy1 = np.searchsorted(self.y, [y-dy-1e-8, y+dy+1e-8])
        xx = self.x[None, ix0:ix1]-x
        yy = self.y[iy0:iy1, None]-y
        inside = (np.abs(xx*c+yy*s) <= self.forward/2+1e-8)
        inside &= np.abs(-xx*s+yy*c) <= self.cross/2+1e-8
        inside &= self.mask[iy0:iy1, ix0:ix1]
        rows, cols = np.nonzero(inside)
        return (rows+iy0)*self.shape[1]+cols+ix0

    def route_counts(self, poses, stride=1):
        counts = np.zeros(self.size, dtype=np.int32)
        for pose in poses[::stride]:
            counts[self.footprint(pose)] += 1
        return counts
