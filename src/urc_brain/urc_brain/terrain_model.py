"""Pure geometry shared by simulation-only terrain filtering and assessment."""
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image


class Terrain:
    def __init__(self, directory):
        p = Path(directory)
        self.meta = json.loads((p/'manifest.json').read_text())
        self.z = (np.asarray(Image.open(p/'height.png'), dtype=float)/65535 *
                  self.meta['height_scale_m'] + self.meta['baseline_elevation_m'] -
                  self.meta['spawn_elevation_m'])
        self.step = self.meta['output_spacing_m']
        self.center = (len(self.z)-1)/2

    def height(self, x, y):
        col, row = x/self.step+self.center, self.center-y/self.step
        if not (math.isfinite(row) and math.isfinite(col) and
                0 <= row < len(self.z)-1 and 0 <= col < len(self.z)-1):
            return math.nan
        r,c = math.floor(row), math.floor(col)
        v,u = row-r,col-c
        return float((1-u)*(1-v)*self.z[r,c] + u*(1-v)*self.z[r,c+1] +
                     (1-u)*v*self.z[r+1,c] + u*v*self.z[r+1,c+1])

    def line_of_sight(self, a, b, base_height=2.0, rover_height=0.8):
        za,zb = self.height(*a)+base_height,self.height(*b)+rover_height
        if not (math.isfinite(za) and math.isfinite(zb)):
            return False
        n = max(3, math.ceil(math.dist(a,b)/(self.step/2)))
        for t in np.linspace(0,1,n)[1:-1]:
            x,y = (1-t)*a[0]+t*b[0],(1-t)*a[1]+t*b[1]
            if self.height(x,y) >= (1-t)*za+t*zb:
                return False
        return True


def rotate(q, vector):
    """Quaternion (x,y,z,w), vector in child coordinates -> parent coordinates."""
    q = np.asarray(q,dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid rotation quaternion')
    q /= norm
    xyz,w = q[:3],q[3]
    v = np.asarray(vector,dtype=float)
    return v + 2*np.cross(xyz,np.cross(xyz,v)+w*v)


def stopped_arrival(xy, target_xy, speed, angular_speed, led, radius):
    values = [*xy,*target_xy,speed,angular_speed,radius]
    return (all(math.isfinite(v) for v in values) and radius > 0 and
            math.dist(xy,target_xy) <= radius and abs(speed)<0.03 and
            abs(angular_speed)<0.05 and led=='flashing_green')
