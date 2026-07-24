from .scene import ScenePCD, SceneMap, SceneTrav


class SceneIsaacsim():
    pcd = ScenePCD()
    #pcd.file_name = 'fast_lio_map_20260715_150306_octomap.pcd'
    pcd.file_name = '3dfactory.pcd'


    map = SceneMap()
    map.resolution = 0.50
    map.ground_h = -0.5
    map.slice_dh = 1.0

    trav = SceneTrav()
    trav.kernel_size = 3
    trav.interval_min = 0.25
    trav.interval_free = 0.45
    trav.slope_max = 0.7
    trav.step_max = 0.6
    trav.standable_ratio = 0.10
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.1
    trav.inflation = 0.5
