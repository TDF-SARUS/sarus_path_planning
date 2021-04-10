from __future__ import division
from __future__ import absolute_import
import numpy as np

def coverage_metrics(area_map, path):
    u"""
    Returns a dict of metrics pertaining to area coverage.
    
    PARAMETERS
    ---
    area_map : -1 (obstacle), 0 valued 2 dim ndarray
    path : [(x1,y1),..., (xm,ym)]
    """
    path = np.array(path).T
    assert path.shape[0] == 2
    x,y = path
    
    p_len = path.shape[1]
    obs_map = area_map == -1
    obs_points = obs_map.sum()
    total_points_nobs = (~obs_map).sum()
    
    vis_map = obs_map.copy()
    vis_map[x,y] = True
    vis_points = vis_map.sum() - obs_points
    
    coverage = vis_points/total_points_nobs
    redundancy = p_len/vis_points - 1
    
    return {
        u"points_to_visit":total_points_nobs,
        u"obstacle_points":obs_points,
        u"points_visited":vis_points,
        u"coverage_path_len":p_len,
        u"coverage":coverage,
        u"redundancy":redundancy,
        u"area_shape":area_map.shape
    }

def fuel_metrics(fuel_paths, fuel_capacity, full_path, area_map):
    u"""
    TODO : Need to complete this function, add more metrics.
    """
    full_path_metrics = coverage_metrics(area_map, full_path)
    fp_len = np.sum([len(fp) for fp in fuel_paths])*2
    return {
        u"refuels" : len(fuel_paths),
        u"fuel_path_len": fp_len,
        u"fuel_capacity": fuel_capacity,
        u"total_redundancy": full_path_metrics[u"redundancy"],
    }