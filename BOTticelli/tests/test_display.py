"""Presentation tests: all sensor fields, exact grids, pipes and current-terminal live views."""
from copy import deepcopy
from io import StringIO

import pytest
from rich.console import Console

from botticelli.display import Watch, render


class TTY(StringIO):
    def isatty(self):
        return True


def printed(data, *, width=80, full=True):
    stream = StringIO()
    console = Console(file=stream, width=width, force_terminal=False, color_system=None)
    console.print(render(data, width, full=full))
    return stream.getvalue()


@pytest.mark.parametrize('data,labels', [
    ({'test':'servo read', 'commanded_degrees':{'S0':90,'S5':135}}, ('CHANNEL','DEG (commanded)','S0','S5','135')),
    ({'test':'motor read', 'commanded_percent':{'M1':-25}}, ('M1','% (commanded)','-25')),
    ({'leds':{'L4':dict(red=12,green=23,blue=34,brightness=45)}}, ('L4','RED','GREEN','BLUE','BRIGHTNESS','45')),
    ({'leds':'3 K10 RGB LEDs and 2 expansion LEDs'}, ('leds','3 K10 RGB LEDs','2 expansion LEDs')),
    ({'board_leds':{'leds':[dict(id=2,red=12,green=23,blue=34,brightness=45)],'extra':9}}, ('BOARD LEDS','ID','2','extra','9')),
    ({'sounds':['beep.wav','start.wav'],'free_bytes':1000000,'playing':False}, ('FILE','beep.wav','start.wav','free bytes','1000000','no')),
    ({'sensor':'imu','frame_id':123,'accel_mg':[1,-2,1000]}, ('IMU','frame 123','X mg','Y mg','Z mg','-2','1000')),
    ({'sensor':'compass','frame_id':44,'heading_deg':270,'field_ut':[1,-2,45]}, ('COMPASS','frame 44','HEADING deg','270','X uT','Y uT','Z uT')),
    ({'sensor':'bump','frame_id':23,'direction':1,'accel_mg':[100,200,1000]}, ('IMU / IMPACTS','frame 23','impact 1 X+','1000')),
    ({'sensor':'bump','state':'NO EVENTS','events_received':0,'seconds':5}, ('NO EVENTS','events received','0','seconds','5')),
])
def test_tables_preserve_values_without_json_or_ansi(data, labels):
    original = deepcopy(data)
    text = printed(data)
    assert data == original
    for label in labels: assert label in text
    assert '\x1b' not in text and not text.lstrip().startswith('{')


@pytest.mark.parametrize('name,model,title', [
    ('lidar', 'DFRobot SEN0682 64x8 DTOF LiDAR', 'LIDAR'),
    ('intensity', 'DFRobot SEN0682 64x8 DTOF LiDAR', 'LIDAR / INTENSITY'),
    ('imu', 'DFRobot SEN0409 LIS2DW12 accelerometer', 'IMU'),
    ('bump', 'DFRobot SEN0409 LIS2DW12 accelerometer', 'IMU / IMPACTS'),
    ('compass', 'DFRobot SEN0619 BMM350 magnetometer', 'COMPASS'),
    ('camera', 'DFRobot SEN0305 HuskyLens K210', 'HUSKYLENS'),
])
def test_sensor_headers_identify_the_physical_device(name, model, title):
    text = printed(dict(sensor=name, device=model, state='WAITING'), width=120)
    assert title in text and text.count(model) == 1


def test_camera_shows_all_block_and_arrow_fields():
    text = printed(dict(sensor='camera',frame_id=12,width=320,height=240,
        blocks=[[101,102,33,44,7,88]],arrows=[[11,22,111,222,9,77]]))
    for label in ('frame 12','320x240 px','blocks 1','arrows 1','BLOCKS','ARROWS','CONF',
                  'W px','H px','X0 px','Y0 px','X1 px','Y1 px','101','102','33','44','7','88','111','222','9','77'):
        assert label in text


def test_age_is_recomputed_and_stale_data_is_not_presented_as_fresh(monkeypatch):
    monkeypatch.setattr('botticelli.display.time.monotonic',lambda:100)
    data=dict(sensor='lidar',frame_id=12,timestamp_ms=12345,_received_at=99.25,distance_mm=[1000]*512)
    text=printed(data,full=False)
    for field in ('frame 12','device ms 12345','age 0.750s STALE','8x64 mm'):
        assert field in text
    assert 'fresh' not in text


@pytest.mark.parametrize('width',[24,40,80,180])
def test_exact_grid_preserves_all_512_values_and_coordinates_at_any_width(width):
    values=list(range(50000,50512))
    text=printed(dict(sensor='intensity',frame_id=3,intensity_raw=values),width=width)
    cells={}
    columns=[]
    for line in text.splitlines():
        if not line.startswith('|'): continue
        parts=[v.strip() for v in line.split('|')[1:-1]]
        if parts[0]=='r/c': columns=[int(c) for c in parts[1:]]
        elif columns and parts[0].isdigit():
            for col,value in zip(columns,parts[1:]): cells[int(parts[0]),col]=int(value)
    assert cells=={(r,c):values[r*64+c] for r in range(8) for c in range(64)}
    assert max(map(len,text.splitlines())) <= width
    assert all(ord(c)<128 for c in text)


@pytest.mark.parametrize('width',[32,80])
def test_compact_map_uses_every_cell_and_explains_scale_and_invalid_returns(width):
    values=[100,5000,0,65535]*128
    text=printed(dict(sensor='lidar',distance_mm=values),width=width,full=False)
    cells=''.join(line.split('|')[1] for line in text.splitlines() if len(line)>2 and line[0].isdigit() and '|' in line)
    assert len(cells)==512
    assert set(cells)=={'@','.', '-', '?'}
    assert '100..5000' in text and '--full' in text
    assert all(ord(c)<128 for c in text)


def test_device_text_cannot_inject_terminal_controls_or_rich_markup():
    text=printed(dict(name='[red]bot[/red]\x1b[2J\nreset'))
    assert '[red]bot[/red]' in text
    assert '\\x1b' in text and '\x1b' not in text and '\\x0a' in text


def test_long_metadata_is_folded_without_ellipsis_or_data_loss():
    value='x'*200
    text=printed({'long_metadata':value},width=40)
    assert text.count('x')==200
    assert '\u2026' not in text


def test_live_view_stays_in_current_terminal_and_restores_cursor_on_exception(monkeypatch):
    monkeypatch.setenv('TERM','xterm-256color')
    stream=TTY()
    console=Console(file=stream,width=80,height=30,force_terminal=True,color_system=None)
    view=Watch(console=console)
    with pytest.raises(ValueError):
        with view:
            view.update({'imu':dict(sensor='imu',accel_mg=[1,2,3])})
            view.update({'imu':dict(sensor='imu',accel_mg=[4,5,6])})
            assert view.live is not None
            raise ValueError('exit')
    text=stream.getvalue()
    assert '\x1b[?1049' not in text  # never switch to an alternate screen
    assert '\x1b[?25h' in text and view.live is None


def test_pipes_receive_each_plain_snapshot_without_cursor_controls():
    stream=StringIO()
    console=Console(file=stream,width=80,force_terminal=False,color_system=None)
    with Watch(console=console) as view:
        view.update({'imu':dict(sensor='imu',accel_mg=[1,2,3])})
        view.update({'imu':dict(sensor='imu',accel_mg=[4,5,6])})
    assert '\x1b' not in stream.getvalue()
    assert stream.getvalue().count('IMU')==2


def test_oversized_live_view_falls_back_without_hiding_rows(monkeypatch):
    monkeypatch.setenv('TERM','xterm-256color')
    stream=TTY()
    console=Console(file=stream,width=40,height=5,force_terminal=True,color_system=None)
    values=list(range(1000,1512))
    with Watch(console=console,full=True) as view:
        view.update({'lidar':dict(sensor='lidar',distance_mm=values)})
        assert view.live is None
    text=stream.getvalue()
    assert '\x1b' not in text
    assert all(str(v) in text for v in values)


def test_forced_color_on_a_pipe_does_not_enable_cursor_updates(monkeypatch):
    monkeypatch.setenv('TERM','xterm-256color')
    stream=StringIO()
    console=Console(file=stream,width=80,height=30,force_terminal=True,color_system=None)
    with Watch(console=console) as view:
        view.update({'imu':dict(sensor='imu',accel_mg=[1,2,3])})
        assert view.live is None
    assert '\x1b' not in stream.getvalue()


def test_color_heatmap_and_cell_probe_keep_exact_measurements(monkeypatch):
    monkeypatch.setenv('TERM','xterm-256color')
    monkeypatch.delenv('NO_COLOR',raising=False)
    stream=TTY()
    console=Console(file=stream,width=80,height=40,force_terminal=True,color_system='truecolor',record=True)
    values=[1000]*512
    values[0],values[1],values[2],values[3]=0,99,400,5000
    with Watch(console=console,cell=(0,3)) as view:
        view.update({'lidar':dict(sensor='lidar',distance_mm=values)})
    raw=stream.getvalue()
    plain=console.export_text()
    assert '48;2;' in raw  # true-color background tiles
    assert 'Closest in-range: 0.400 m (400 mm) at row 0, col 2' in plain
    assert 'Cell row 0, col 3: 5.000 m (5000 mm)' in plain
    assert 'below 1' in plain and 'CAUTION' in plain
    assert '\x1b[?1049' not in raw


def test_no_color_disables_tiles_but_keeps_measurements(monkeypatch):
    monkeypatch.setenv('NO_COLOR','1')
    stream=TTY()
    console=Console(file=stream,width=80,height=40,force_terminal=True,color_system='truecolor',record=True)
    with Watch(console=console) as view:
        view.update({'lidar':dict(sensor='lidar',distance_mm=[400]*512)})
    assert '48;2;' not in stream.getvalue()
    text=console.export_text()
    assert '@'*64 in text
    assert 'Closest in-range: 0.400 m' in text


def test_unknown_ranges_never_produce_a_false_closest_distance():
    data=dict(sensor='lidar',distance_mm=[0,99,65535,0]*128)
    text=printed(data,full=False)
    assert 'Closest in-range: UNKNOWN' in text
    assert 'below 128' in text and 'above 128' in text
    assert 'CAUTION' in text
