import math
import numpy as np
from equipo_jar.estrategia import Grid, Victims, speed_limit


def observe(v, point, start=0):
    reports = []
    for stamp in np.arange(start,start+1,.1):
        result = v.observe(point,float(stamp))
        if result is not None:
            reports.append(result)
    return reports


def test_revisit_from_other_view_does_not_spend_budget():
    v = Victims(2)
    assert len(observe(v,(1,2))) == 1
    assert observe(v,(1.15,1.9),4) == []
    assert len(observe(v,(2.3,2),6)) == 1
    assert v.count == 2
    assert observe(v,(5,5),8) == []


def test_nan_duplicate_frames_and_unstable_estimates_do_not_report():
    v = Victims(3)
    for _ in range(20):
        assert v.observe((1,2),1) is None
    assert v.observe((math.nan,2),2) is None
    for i in range(20):
        assert v.observe((1+(i%2)*.3,2),2+i*.1) is None
    assert v.count == 0


def test_fixed_report_anchor_prevents_drift_chain():
    v = Victims(3)
    observe(v,(1,1))
    observe(v,(1.6,1),4)
    assert tuple(v.tracks[0].center) == (1,1)
    assert v.count == 1


def test_braking_distance_including_latency():
    for distance in np.linspace(0,2,100):
        speed = speed_limit(distance,.12,.06,.25,.4)
        assert 0 <= speed <= .12
        if speed:
            assert speed*.4+speed*speed/(2*.25) <= distance-.06+1e-12
    assert speed_limit(math.nan,.12,.06,.25,.4) == 0


def test_rotated_map_roundtrip_and_unknown_boundary_blocking():
    g = Grid(np.zeros((20,20)),.1,(2,-1,.7),.15)
    for cell in ((2,3),(10,10),(17,16)):
        assert g.cell(*g.point(cell)) == cell
    assert not g.free[0,0]
    values = np.zeros((20,20))
    values[:,10] = -1
    g = Grid(values,.1,(0,0,0),.15)
    costs,_ = g.paths((10,5))
    assert (10,15) not in costs


def test_dynamic_obstacle_detour_and_expiry():
    g = Grid(np.zeros((30,30)),.1,(0,0,0),.12)
    g.add_obstacle(g.point((15,15)),4)
    g.refresh(1)
    costs,parents = g.paths((15,5))
    assert (15,25) in costs
    path = g.route((15,5),(15,25),parents)
    assert all(math.dist(p,g.point((15,15))) > .12 for p in path)
    g.refresh(5)
    assert g.free[15,15]


def test_diagonal_cannot_cut_blocked_corner():
    data = np.zeros((6,6))
    data[2,3] = data[3,2] = 100
    g = Grid(data,.1,(0,0,0),0)
    _,parents = g.paths((2,2))
    assert parents.get((3,3)) != (2,2)
