u"""
Some helper functions that are often used.
"""
from __future__ import absolute_import
import numpy as np
import matplotlib.pyplot as plt
from .constants import OB, NO
from pathlib import Path
from PIL import Image

RES = [(32, 32),(50, 50),(50, 144),(144, 255),(256,256)]

def adjacency_test(path, exactly_one=True):
    u"""
    Checks all points in a path for L1 adjacency.
    """
    prev_point = path[0]
    for i,point in enumerate(path[1:]):
        x,y = prev_point
        x_,y_ = point 
        
        dist = np.abs(x - x_) + np.abs(y - y_)
        prev_point = point
        if exactly_one and dist == 1:
            continue
        elif dist <= 1:
            continue
        else:
            return i - 1
    return True

def generate_no_obs_area_map(resolutions=RES):
    u"""
    resolutions : list of tuples [(rows, cols)]
    """
    area_maps = []
    for res in resolutions:
        area_maps.append(np.zeros(res))
    return area_maps

def generate_point_obstacles(area_map, p=0.5):
    u"""
    Adds point obstacles to the area_map with the given
    probability `p`, if `p==1` then the entire map will
    be covered.
    """
    area_map = area_map.copy()
    area_map[np.random.rand(*area_map.shape)<p] =  -1
    return area_map

def get_area_map(path, area=0, obs=-1):
    u"""
    path : path to area map png, png should have only 
    0 255 as values.
    returns area_map with cell values
    obstacle : OBS
    non obstacle : NOB
    """
    am = np.array(Image.open(path))
    ma = np.array(am).mean(axis=2) == 255
    am = np.int8(np.zeros(ma.shape))
    am[ma] = area
    am[~ma]  = obs
    return am

def imshow_scatter(path, color=u"orange", alpha=1, s=20):
    u"""
    Prints the points in the path
    """
    x,y = np.array(path).T
    plt.scatter(y,x, color=color, alpha=alpha, s=s)

def imshow(area_map,r=1,c=1,i=1,figsize=(5,5),cmap=u"viridis"):
    u"""
    Display with no interpolation.
    """
    if r < 2 and c < 2 or i == 1:
        plt.figure(figsize=figsize)
    plt.subplot(r,c,i)
    ax = plt.imshow(area_map,interpolation=u'none',cmap=cmap)
    plt.axis(u'off');
    return ax

def plot(cp, alpha=0.8, color=u"lightblue"):
    u"""
    Plot coverage path as a line.
    """ 
    cp = np.array(cp)
    x,y = cp.T
    plt.plot(y,x,alpha=alpha,color=color)

def get_random_coords(area_map,n=2,obs=-1):
    u"""
    Return random coords from the map
    where there are no obstacles.
    
    n : number of random points
    obs : obstacle value on the area map
    """
    r = lambda x : np.random.randint(0,x)
    b1, b2 = area_map.shape
    coords = []
    for i in xrange(n):
        while True:
            p = (r(b1), r(b2))
            if area_map[p] != obs:
                coords.append(p)
                break
    return coords

def set_val(area_map, coords, val):
    u"""
    Set `val` at given `coords` on
    the `area_map`
    
    area_map : 2D numpy array
    coords : list of (x,y) tuples
    val : int of value to set
    """
    x, y = np.array(coords).T
    area_map[x,y] = val

def is_bounded(coord, shape):
    u"""
    Checks if a coord (x,y) is within bounds.
    """
    x,y = coord
    g,h = shape
    lesser = x < 0 or y < 0
    greater = x >= g or y >= h
    if lesser or greater:
        return False
    return True

def is_valid(coord, area_map, obstacle = -1):
    u"""
    Check is a coord (x,y) is bounded and not
    on an obstacle.
    """
    coord = tuple(coord)
    is_b = is_bounded(coord, area_map.shape)

    if is_b:
        is_on_obs = False
        if isinstance(obstacle, list):
            for obs in obstacle:
                is_on_obs |= area_map[coord] == obs
        else:
            is_on_obs  = area_map[coord] == obstacle
        if not is_on_obs:
            return True
    return False

def get_all_area_maps(folder_path):
    u"""
    Returns size sorted list of area maps.
    
    folder_path : path to the folder contiaining the maps (.png)
    """
    ams = []
    for path in Path(folder_path).iterdir():
        try:
            ams.append(get_area_map(path))
        except:
            continue
            
    am_idx = np.array([am.size for am in ams]).argsort()
    return list(np.array(ams)[am_idx])

def get_drone_map(A, i, obstacle=OB, coverage=NO):
    u"""
    Returns area map for a single drone 
    from the assignment matrix.
    
    PARAMETERS
    ---
    A : assignment matrix obtained from darp.
    i : the drone number (cell value of A).
    obstacle : value to assign the obstacle
    coverage : value to assign the coverage area
    """
    am = A.copy()
    x,y = np.where(am != i)
    am[x,y] = obstacle
    x,y = np.where(am == i)
    am[x,y] = coverage
    return am 