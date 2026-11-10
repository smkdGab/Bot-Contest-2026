"""Battle rule tests use offline worlds and synthetic input only."""
from copy import deepcopy
from dataclasses import asdict
import math
import socket
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import Point

from botticelli.battle import Battle
from botticelli.battle_config import generate
from botticelli.battle_geometry import Geometry, relative_hit, segment_blocked
from botticelli.battle_policy import BattleObservation, Detection, HuntingPolicy
from botticelli.battle_runtime import BattleSession, BattleWindow, replay_battle, run_battle
from botticelli.control import DriveCommand, Input
from botticelli.recording import Recorder, read
from botticelli.config import Config


def setup(poses=((1,2,0),(3,2,180)), **extra):
    return generate({'arena': {'polygon': [[0,0],[6,0],[6,6],[0,6]]},
        'car': {'toothpick_count': 1, 'toothpicks': [
            {'position': [.25,0,.28], 'length': .1, 'direction_deg': 0, 'elevation_deg': 0}],
            'balloon': {'radius': .07, 'position': [-.14,0,.28]}},
        'cars': {str(i): {'pose': list(p)} for i,p in enumerate(poses)}, **extra},
        seed=21, opponents=len(poses)-1)


@pytest.mark.parametrize('n', [1,5,8])
def test_count_streams_and_repeatable_battle(n):
    s = generate(seed=42,opponents=n)
    assert s == generate(seed=42,opponents=n)
    assert len(s['cars']) == n+1
    assert all(1 <= len(c['toothpicks']) <= 3 for c in s['cars'])
    a,b = Battle(s),Battle(deepcopy(s))
    for _ in range(8):
        assert a.step() == b.step()
    assert a.snapshot() == b.snapshot()
    other = generate(seed=42,opponents=1)
    assert s['arena'] == other['arena']
    for c in s['cars'][:2]:
        assert c['sensor_seed'] == other['cars'][c['id']]['sensor_seed']
        assert c['toothpicks'] == other['cars'][c['id']]['toothpicks']


def test_random_count_and_overrides():
    assert {len(generate(seed=i)['cars'])-1 for i in range(20)} == {1,2,3,4,5}
    s = generate({'opponents': {'min':2,'max':2}, 'car': {'toothpick_count':1},
        'cars': {'0': {'length':.4,'balloon':{'radius':.08},'toothpick':{'length':.11}}}},seed=3)
    assert len(s['cars']) == 3
    assert all(len(c['toothpicks']) == 1 for c in s['cars'])
    assert s['cars'][0]['length'] == .4
    assert s['cars'][0]['toothpicks'][0]['length'] == .11
    assert s['cars'][0]['balloon']['radius'] == .08
    assert type(generate()['seed']) is int


@pytest.mark.parametrize('shape', ['rectangle','convex','l'])
def test_boundary_agrees_with_spawning_and_ranges(shape):
    s = generate({'arena': {'shapes': {k:int(k==shape) for k in ('rectangle','convex','l')}}},seed=5)
    geom = Geometry(s)
    assert geom.boundary.is_valid and s['arena']['shape'] == shape
    assert all(geom.boundary.covers(Point(*c['pose'][:2])) for c in s['cars'])
    x,y = geom.boundary.representative_point().coords[0]
    for yaw in np.linspace(-math.pi,math.pi,20):
        direction = np.array([[math.cos(yaw),math.sin(yaw),0]])
        distance = geom.ranges([x,y,.1],direction,[])[0]
        assert math.isfinite(distance)
        assert geom.boundary.boundary.distance(Point(x+distance*direction[0,0],y+distance*direction[0,1])) < 1e-8


@pytest.mark.parametrize('data', [
    {'opponents':0}, {'opponents':1.5}, {'opponents':{'min':5,'max':1}},
    {'arena':{'polygon':[[0,0],[1,1],[1,0],[0,1]]}},
    {'arena':{'width':.2,'height':.2,'shapes':{'rectangle':1,'convex':0,'l':0}},'generation_attempts':4},
    {'arena':{'passage_min':10,'shapes':{'rectangle':0,'convex':0,'l':1}}},
    {'cars':{'99':{}}}, {'car':{'toothpick_count':0}},
    {'car':{'balloon':{'height':.01}}}, {'car':{'toothpick':{'position':[0,0,-.1]}}},
    {'car':{'sensors':{'noise_std':-1}}}, {'damage':{'reference_speed':0}}, {'unknown':1},
])
def test_invalid_or_overcrowded_setup(data):
    with pytest.raises((ValueError,TypeError)):
        generate(data,seed=1)


def test_overlap_blocked_tooth_and_initial_pop_rejected():
    with pytest.raises(ValueError,match='Cannot fit'):
        setup(poses=((1,2,0),(1,2,0)))
    with pytest.raises(ValueError,match='Cannot fit'):
        setup(car={'toothpick_count':1,'toothpicks':[{'position':[0,0,.1],'length':.1}]})
    with pytest.raises(ValueError,match='Cannot fit'):
        setup(poses=((1,2,0),(1.5,2,0)))


def test_inward_corner_collision_sensing_and_reverse():
    w = Battle(setup(poses=((1.5,3,0),(.8,4,0)),arena={
        'polygon':[[0,0],[6,0],[6,2],[2,2],[2,6],[0,6]]}))
    assert w.geometry.ranges([1.5,3,.1],np.array([[1,0,0]]),[])[0] == pytest.approx(.5)
    assert segment_blocked([1.5,3,.1],[3,3,.1],w.geometry.solids)
    for _ in range(80): w.step({0:DriveCommand(1)})
    assert w.cars[0].pose[0] < 1.751 and w.cars[0].health == pytest.approx(50)
    x = w.cars[0].pose[0]
    for _ in range(5): w.step({0:DriveCommand(-1)})
    assert w.cars[0].pose[0] < x


def test_relative_sweeps_and_height_miss():
    assert relative_hit([0,0,.28],[2,0,.28],np.array([1,0,.28]),np.array([1,0,.28]),.07) == pytest.approx(.465)
    assert relative_hit([0,0,.1],[2,0,.1],np.array([1,0,.28]),np.array([1,0,.28]),.07) is None
    assert relative_hit([0,0,.28],[1,0,.28],np.array([1,0,.28]),np.array([0,0,.28]),.07) == pytest.approx(.465)


def test_sensor_only_policy_approaches_and_pops_exposed_balloon():
    w = Battle(setup(poses=((1,2,0),(2,2,0))))
    assert w.observations[0].balloons
    assert set(asdict(w.observations[0])) == {'timestamp','distances','balloons','collision','fresh_seconds'}
    assert set(asdict(w.observations[0].balloons[0])) == {'bearing','radius_angle','elevation'}
    events = []
    for _ in range(200):
        events += w.step({0:w.policies[0].step(w.observations[0],w.time)})
        if w.finished: break
    pops = [e for e in events if e['type']=='pop']
    assert len(pops)==1 and pops[0]['attacker']==0 and pops[0]['victim']==1
    assert w.finished and w.winner==0
    assert w.cars[1].eliminated and w.cars[1].popped and w.commands[1]==DriveCommand()


def test_shaft_height_misses_and_shielding():
    s = setup(poses=((1,2,0),(3,2,0)))
    s['cars'][0]['toothpicks'][0].update(position=[.25,0,.28],length=.8)
    w = Battle(s)
    w.cars[1].pose = [1.75,2,0]
    assert not any(e['type']=='pop' for e in w.step({}))
    w.cars[1].pose = [2.19,2,0]
    w.cars[0].spec['toothpicks'][0]['position'][2] = .6
    assert not any(e['type']=='pop' for e in w.step({}))
    s = setup(poses=((1,2,0),(2,2,0)))
    s['cars'][1]['height'] = .5
    s['cars'][1]['balloon']['position'] = [.4,0,.28]
    w = Battle(s)
    assert not w.observations[0].balloons
    w.cars[0].pose=[1.5,2,0]
    w.cars[0].spec['toothpicks'][0]['length'] = .65
    assert not any(e['type']=='pop' for e in w.step({}))


@pytest.mark.parametrize('speed,health', [(.1,100),(.6,50),(1.2,0)])
def test_wall_damage_attempted_velocity_and_continuous_contact(speed,health):
    w = Battle(setup(poses=((5.5,2,0),(1,4,0)),car={'max_speed':speed}))
    events=[]
    for _ in range(150): events += w.step({0:DriveCommand(1)})
    assert w.cars[0].health == pytest.approx(health,abs=1e-7)
    assert w.cars[0].pose[0] <= 5.75
    assert len([e for e in events if e['type']=='contact' and e['a']==0]) == 1
    if health == 0:
        assert w.cars[0].causes == ('destroyed',) and w.commands[0]==DriveCommand()


def test_separate_impacts_rearm():
    w=Battle(setup(poses=((5.5,2,0),(1,4,0))))
    for _ in range(40): w.step({0:DriveCommand(1)})
    assert w.cars[0].health == pytest.approx(50)
    for _ in range(10): w.step({0:DriveCommand(-1)})
    assert w.cars[0].pose[0] < 5.7
    for _ in range(30): w.step({0:DriveCommand(1)})
    assert w.cars[0].health == pytest.approx(0)


def test_simultaneous_crash_draw_without_order_bias():
    s=setup(poses=((2,2,0),(3,2,180)))
    a,b=Battle(s),Battle(deepcopy(s))
    b.cars.reverse()
    for _ in range(50):
        ea,eb=a.step({0:DriveCommand(1),1:DriveCommand(1)}),b.step({0:DriveCommand(1),1:DriveCommand(1)})
        assert ea==eb
        if a.finished: break
    assert a.finished and a.winner is None and all(c.health==0 and c.eliminated for c in a.cars)
    assert a.cars[0].pose[0] < a.cars[1].pose[0]


def test_wreck_is_stationary_obstacle_and_sliding_does_no_damage():
    w=Battle(setup(poses=((1,2,0),(2,2,0),(4,4,0))))
    wreck=w.cars[1]
    wreck.eliminated,wreck.health=True,0
    position=list(wreck.pose)
    for _ in range(100): w.step({0:DriveCommand(1),1:DriveCommand(1)})
    assert wreck.pose==position and w.cars[0].pose[0]<1.71 and not w.finished
    w=Battle(setup(poses=((5.88,2,90),(1,4,0))))
    for _ in range(60): w.step({0:DriveCommand(1)})
    assert w.cars[0].health==100 and w.cars[0].pose[1]>2.5


def test_policy_expiry_and_bounded_reverses():
    p=HuntingPolicy()
    obs=BattleObservation(0,(5,)*7,(Detection(.2,.1,0),))
    assert p.step(obs,0).throttle>0 and p.step(obs,1)==DriveCommand()
    p.step(BattleObservation(.6,(5,)*7,()),.6)
    assert p.target is None
    for now in (1,1.76): assert p.step(BattleObservation(now,(.05,)*7,()),now).throttle<0
    assert p.step(BattleObservation(2.52,(.05,)*7,()),2.52)==DriveCommand()


def test_pause_loss_rearm_and_release_a():
    s=BattleSession(setup())
    s.update(Input(0,True),0)
    assert s.paused and s.world.time==0
    s.update(Input(.02,True),.02,resume=True)
    assert not s.paused
    s.update(Input(.04,True,throttle=1,a=True),.04)
    assert s.world.commands[0].throttle==1
    s.update(Input(.06,True),.06)
    assert not s.paused and s.world.commands[0].throttle==0
    before=s.world.time
    s.update(Input(.06,True,a=True),1)
    assert s.paused and s.world.time==before
    s.update(Input(1.02,True,a=True,generation=1),1.02,resume=True)
    assert s.paused
    s.update(Input(1.04,True,generation=1),1.04,resume=True)
    assert not s.paused
    s.update(Input(1.06,True,generation=1),1.06,pause=True)
    assert s.paused
    s.world.cars[0].eliminated=True
    s.update(Input(1.08,True,generation=1),1.08,resume=True)
    assert not s.paused and s.world.commands[0].throttle==0


def test_full_setup_recording_and_network_free_replay(tmp_path,monkeypatch):
    path=tmp_path/'battle.jsonl'
    args=SimpleNamespace(seconds=.1,headless=True,config=None,seed=3,opponents=1,record=str(path),screenshot=None)
    run_battle(args)
    records=list(read(path)); metadata=records[0]
    assert metadata['schema']==2 and metadata['setup']==generate(seed=3,opponents=1)
    assert {'snapshot','commands','pause'} <= {r['type'] for r in records}
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('Replay opened socket'))
    monkeypatch.setattr(Battle,'__init__',lambda *a,**k:pytest.fail('Replay constructed physics'))
    replay_battle(SimpleNamespace(headless=True),metadata,iter(records[1:]))
    with pytest.raises(FileExistsError): Recorder(str(path),Config(),'battle',setup=metadata['setup'])


@pytest.mark.parametrize('shape',['rectangle','convex','l'])
def test_window_actual_outline_and_wrecks(shape,tmp_path):
    s=generate({'arena':{'shapes':{k:int(k==shape) for k in ('rectangle','convex','l')}}},seed=2)
    w=Battle(s)
    w.cars[0].eliminated,w.cars[0].health=True,0
    w.cars[0].causes=('destroyed',)
    window=BattleWindow()
    try:
        window.draw(s,w.snapshot(),'Paused')
        window.pg.image.save(window.screen,str(tmp_path/f'{shape}.png'))
        assert window.screen.get_at((10,10))[:3]==(9,13,19)
    finally: window.close()


def test_simultaneous_pops_draw_and_independent_crash_damage():
    s=setup(poses=((1,2,0),(2,2,180)))
    for c in s['cars']: c['toothpicks'][0]['length']=.64
    w=Battle(s)
    events=[]
    for _ in range(30):
        events += w.step({0:DriveCommand(1),1:DriveCommand(1)})
        if w.finished: break
    assert len([e for e in events if e['type']=='pop'])==2
    assert all(c.popped and c.eliminated for c in w.cars)
    assert w.finished and w.winner is None
    # Arrange a front-to-rear hit with a sphere touch at exactly chassis
    # contact. Both scoring mechanisms must survive the same update.
    s=setup(poses=((1,2,0),(2,2,0)))
    s['cars'][0]['toothpicks'][0].update(position=[.053,0,.28],length=.04)
    w=Battle(s)
    w.cars[0].pose=[1.6,2,0]
    events=[]
    for _ in range(120):
        events += w.step({0:DriveCommand(1)})
        if w.finished: break
    assert any(e['type']=='damage' for e in events)
    assert any(e['type']=='pop' for e in events)
    assert {'damage','pop'} <= {e['type'] for e in w.events}


def test_self_pop_and_balloon_wall_contact_are_ignored():
    s=setup()
    s['cars'][0]['toothpicks'][0].update(length=.39,direction_deg=180)
    w=Battle(s)
    assert not any(e['type']=='pop' for e in w.step({}))
    s=setup(poses=((.4,2,0),(4,4,0)),car={'max_speed':.1})
    w=Battle(s)
    for _ in range(200): w.step({0:DriveCommand(-1)})
    assert not w.cars[0].popped and w.cars[0].health==100


def test_angular_contact_speed_contributes_to_damage():
    # A surface point on the front of a turning car has lateral speed even
    # though rear-axle lateral speed is zero.
    velocity=Battle._point_velocity([0,0,0],[.01,0,.02],[.25,.1],.02)
    assert velocity[1] == pytest.approx(.25)
    assert velocity[0] == pytest.approx(.4)


def test_configuration_file_and_exact_attachment_list():
    from botticelli.battle_config import load_battle
    assert len(load_battle('config/battle.yaml',seed=1,opponents=8)['cars'])==9
    with pytest.raises(ValueError,match='conflicts'):
        generate({'car':{'toothpick_count':1,'toothpicks':[
            {'position':[.25,0,.28],'length':.1}, {'position':[.25,.03,.28],'length':.1}]}},seed=1)


def test_keyboard_restart_new_and_repeat_are_paused(tmp_path,monkeypatch):
    import pygame
    import botticelli.battle_runtime as runtime
    import botticelli.dashboard as dashboard
    calls=[]; drawn=[]
    def loader(path, **kwargs):
        calls.append(kwargs)
        return generate(seed=len(calls),opponents=1)
    class Window:
        def draw(self,snapshot_setup,snapshot,state,controller):
            drawn.append((deepcopy(snapshot_setup),snapshot,state))
        def close(self): pass
    class Reader:
        message='test'
        def __init__(self,config): pass
        def poll(self,events):
            import time
            return Input(time.monotonic(),True)
    frames=iter([
        [SimpleNamespace(type=pygame.KEYDOWN,key=pygame.K_RETURN,mod=0)],
        [SimpleNamespace(type=pygame.KEYDOWN,key=pygame.K_r,mod=0)],
        [SimpleNamespace(type=pygame.KEYDOWN,key=pygame.K_r,mod=pygame.KMOD_SHIFT)],
        [SimpleNamespace(type=pygame.QUIT)],
    ])
    monkeypatch.setattr(runtime,'load_battle',loader)
    monkeypatch.setattr(runtime,'BattleWindow',Window)
    monkeypatch.setattr(dashboard,'GamepadReader',Reader)
    monkeypatch.setattr(pygame.event,'get',lambda:next(frames))
    path=tmp_path/'restarts.jsonl'
    runtime.run_battle(SimpleNamespace(seconds=None,headless=False,config=None,seed=42,
        opponents=1,record=str(path),screenshot=None))
    assert len(calls)==2 and calls[0]['seed']==42 and 'seed' not in calls[1]
    assert drawn[0][0]['seed']==1 and drawn[0][1]['time']>0
    assert drawn[1][0] == drawn[2][0] and drawn[1][0]['seed']==2
    assert drawn[1][1]['time']==drawn[2][1]['time']==0
    assert 'Enter' in drawn[1][2] and 'Enter' in drawn[2][2]
    records=list(read(path))
    assert len([r for r in records if r['type']=='setup'])==2


def test_pose_ranges_and_sensor_loss_are_reproducible_and_safe():
    data={'car':{'sensors':{'drop_probability':1}},'cars':{'0':{
        'pose':[{'min':1,'max':2},1,{'min':0,'max':90}]}}}
    s=generate(data,seed=7,opponents=1)
    assert s==generate(data,seed=7,opponents=1)
    assert 1<=s['cars'][0]['pose'][0]<=2 and s['cars'][0]['pose'][1]==1
    w=Battle(s)
    for _ in range(4): w.step()
    assert all(cmd==DriveCommand() for cmd in w.commands.values())
    # A dropped frame retains the old timestamp instead of becoming fresh.
    assert all(obs.timestamp<0 for obs in w.observations.values())


def test_other_car_occlusion_and_spectating_without_controller():
    s=setup(poses=((1,2,0),(2,2,0),(3,2,0)))
    s['cars'][1]['height']=.5
    w=Battle(s)
    w.cars[1].eliminated=True
    w.next_scan={c.id:0 for c in w.cars}; w._sense()
    assert not w.observations[0].balloons
    session=BattleSession(setup(poses=((1,2,0),(3,2,0),(4,4,0))))
    session.world.cars[0].eliminated=True
    session.update(Input(0),0,resume=True)
    session.update(Input(.02),.02)
    assert not session.paused and session.world.time==pytest.approx(.04)


def test_heading_wrap_uses_short_arc_and_sensing_keeps_eight_hz():
    from botticelli.battle_geometry import interpolate_pose
    pose=interpolate_pose([0,0,math.pi-.01],[0,0,-math.pi+.01],.5)
    assert pose[2]==pytest.approx(math.pi)
    w=Battle(setup())
    scans=1; stamp=w.observations[0].timestamp
    for _ in range(50):
        w.step({})
        current=w.observations[0].timestamp
        if current != stamp:
            scans+=1; stamp=current
    assert scans==9  # initial frame plus eight frames during the first second


@pytest.mark.parametrize('data',[
    {'arena':{'polygon':[[0,0,0],[4,0,0],[4,4,0],[0,4,0]]}},
    {'cars':{'0':[]}}, {'car':{'toothpicks':{}}},
    {'car':{'toothpicks':[{'position':[.25,0,.28]}]}},
])
def test_malformed_geometry_settings_fail_clearly(data):
    with pytest.raises(ValueError): generate(data,seed=1,opponents=1)


def test_random_mounts_cover_every_edge_and_keep_explicit_positions_fixed():
    balloon_sides=set(); tooth_sides=set(); coordinates=[]
    def side(position, spec, *, balloon=False):
        rear=-(spec['length']-spec['wheelbase'])/2; front=rear+spec['length']
        x,y=position[:2]
        if balloon:
            if x < rear: return 'rear'
            if x > front: return 'front'
            return 'left' if y>0 else 'right'
        if abs(x-rear)<1e-9: return 'rear'
        if abs(x-front)<1e-9: return 'front'
        return 'left' if y>0 else 'right'
    for seed in range(20):
        s=generate(seed=seed,opponents=1)
        for spec in s['cars']:
            balloon_sides.add(side(spec['balloon']['position'],spec,balloon=True))
            coordinates.append(tuple(spec['balloon']['position']))
            for t in spec['toothpicks']:
                edge=side(t['position'],spec); tooth_sides.add(edge)
                outward={'front':0,'rear':180,'left':90,'right':-90}[edge]
                assert abs((t['direction_deg']-outward+180)%360-180)<=30
    assert balloon_sides==tooth_sides=={'front','rear','left','right'}
    assert len(set(coordinates))==len(coordinates)
    for seed in range(3):
        spec=generate({'car':{'balloon':{'position':[-.15,.05,.28]},
            'toothpick_count':1,'toothpick':{'position':[.25,.04,.28],'direction_deg':12}}},
            seed=seed,opponents=1)['cars'][0]
        assert spec['balloon']['position']==[-.15,.05,.28]
        assert spec['toothpicks'][0]['position']==[.25,.04,.28]
        assert spec['toothpicks'][0]['direction_deg']==12


def test_configured_mount_sides_and_invalid_mounts():
    s=generate({'car':{'balloon':{'mount':'rear'},'toothpick':{'mount':'front','spread_deg':0}}},seed=5,opponents=1)
    for spec in s['cars']:
        assert spec['balloon']['position'][0]<0
        assert all(t['position'][0]==pytest.approx(.25) and t['direction_deg']==0 for t in spec['toothpicks'])
    for car in ({'balloon':{'mount':'ceiling'}},{'toothpick':{'spread_deg':-1}}):
        with pytest.raises(ValueError): generate({'car':car},seed=5,opponents=1)
