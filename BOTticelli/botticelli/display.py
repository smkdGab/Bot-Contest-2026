"""Terminal presentation only: Rich tables/live output and ASCII sensor grids."""
import time
from functools import lru_cache

from rich import box
from rich.console import Console, Group
from rich.live import Live
from rich.table import Table
from rich.text import Text
from rich.style import Style

@lru_cache(maxsize=1)
def palette():
    """Fixed near-to-far colors, shared by terminal and dashboard."""
    stops = ((255, 210, 70), (45, 185, 165), (55, 85, 155), (80, 25, 110))
    colors = []
    for i in range(256):
        position = i/255*(len(stops)-1)
        index = min(int(position), len(stops)-2)
        colors.append(tuple(round(a+(b-a)*(position-index)) for a,b in zip(stops[index], stops[index+1])))
    return tuple(colors)


def range_color(mm, minimum=100, maximum=5000):
    if mm == 0:
        return (38, 45, 55)
    if not minimum <= mm <= maximum:
        return (190, 70, 130)
    return palette()[min(255, int((mm-minimum)/(maximum-minimum)*255))]


def distance(mm, minimum=100, maximum=5000):
    if mm == 0:
        return 'NO RETURN (0 mm)'
    if not minimum <= mm <= maximum:
        return f'OUT OF RANGE ({mm} mm)'
    return f'{mm/1000:.3f} m ({mm} mm)'


def closest(values, minimum=100, maximum=5000):
    return min(((mm, i) for i, mm in enumerate(values) if minimum <= mm <= maximum), default=None)


def clean(value):
    if value is None:
        return '-'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, float):
        return f'{value:.6g}'
    # Device strings must not inject terminal controls or Rich markup.
    return ''.join(c if c.isprintable() else f'\\x{ord(c):02x}' for c in str(value))


def table(headers, rows, title=None, *, grid=False):
    result = Table(title=Text(clean(title)) if title else None,
                   title_justify='left', box=box.ASCII if grid else None,
                   padding=(0, 0 if grid else 1), pad_edge=grid, highlight=False)
    for header in headers:
        result.add_column(Text(clean(header)), overflow='fold')
    for row in rows:
        result.add_row(*[Text(clean(cell)) for cell in row])
    return result


def fields(data, prefix=''):
    """Flatten metadata without ever printing a dict/list representation."""
    if isinstance(data, dict):
        for name, value in data.items():
            yield from fields(value, f'{prefix}.{name}' if prefix else name)
    elif isinstance(data, (list, tuple)):
        if not data:
            yield prefix, '(none)'
        elif all(not isinstance(v, (dict, list, tuple)) for v in data):
            yield prefix, '  '.join(clean(v) for v in data)
        else:
            for index, value in enumerate(data):
                yield from fields(value, f'{prefix}.{index}')
    else:
        yield prefix.replace('_', ' '), data


@lru_cache(maxsize=512)
def tile_style(rgb):
    return Style(bgcolor=f'rgb({rgb[0]},{rgb[1]},{rgb[2]})',
                 color='black' if sum(rgb) > 380 else 'white', bold=True)


def grid(values, width, *, full, intensity=False, colored=False, cell=None):
    """Preserve wire row/column order; split columns to fit narrow terminals."""
    valid = [v for v in values if v]
    low, high = (min(valid), max(valid)) if valid else (0, 0)
    unit = 'raw' if intensity else 'mm'
    summary = f'8x64 {unit}  nonzero {len(valid)}/512  min {low}  max {high}'
    summary += f'  mean {sum(valid)/len(valid):.1f}' if valid else '  mean -'
    parts = [Text(summary)]
    nearest = closest(values) if not intensity else None
    if not intensity:
        in_range = [v for v in values if 100 <= v <= 5000]
        below, above = sum(0 < v < 100 for v in values), sum(v > 5000 for v in values)
        parts = [Text(f'8x64 mm  in range {len(in_range)}/512  no return {values.count(0)}  below {below}  above {above}')]
        if nearest:
            mm, index = nearest
            parts.append(Text(f'Closest in-range: {distance(mm)} at row {index//64}, col {index%64}', style='bold'))
        else:
            parts.append(Text('Closest in-range: UNKNOWN', style='bold'))
        if below or above:
            parts.append(Text('CAUTION: out-of-range readings are unknown distances, not clear space.', style='bold yellow'))
    selected = cell[0]*64+cell[1] if cell is not None else None
    if selected is not None:
        value = values[selected]
        label = f'{value} raw' if intensity else distance(value)
        parts.append(Text(f'Cell row {cell[0]}, col {cell[1]}: {label}', style='bold'))
    cell_width = max(3, max(len(str(v)) for v in values)) if full else 1
    tile_width = 2 if colored and width >= 134 else 1
    columns = min(64, max(1, (width-6)//(cell_width+1))) if full else min(64, max(1, (width-6)//tile_width))
    if not full:
        if colored:
            legend = Text('Range: ' if not intensity else 'Intensity: ')
            levels = ((low,) if low == high else (low, high)) if intensity else (100, 1000, 2500, 5000)
            for value in levels:
                mm = 100 + (value-low)/max(1, high-low)*4900 if intensity and value else value
                legend.append('  ', style=tile_style(range_color(mm)))
                legend.append(f' {value} raw  ' if intensity else f' {value/1000:g}m  ')
            parts.append(legend)
            parts.append(Text('N nearest; X selected; - no return/zero; ? out of range; --full exact values'))
        else:
            parts.append(Text(f'scale {low}..{high} raw' if intensity else 'scale 100..5000 mm; @ near, . far'))
        if not colored:
            parts.append(Text('- zero; --full shows exact values' if intensity else
                              '- no return; ? outside nominal range; --full shows exact values'))
    for start in range(0, 64, columns):
        end = min(64, start+columns)
        if full:
            parts.append(table(['r/c']+[str(c) for c in range(start, end)],
                [[r]+list(values[r*64+start:r*64+end]) for r in range(8)], grid=True))
        elif colored:
            lines = Text(f'cols {start:02d}..{end-1:02d}\n  +'+ '-'*((end-start)*tile_width)+'+\n')
            for r in range(8):
                lines.append(f'{r} |')
                for c in range(start, end):
                    i, value = r*64+c, values[r*64+c]
                    mm = 100 + (value-low)/max(1, high-low)*4900 if intensity and value else value
                    marker = 'X' if i == selected else 'N' if nearest and i == nearest[1] else '-' if value == 0 else '?' if not intensity and not 100 <= value <= 5000 else ' '
                    lines.append(marker+' '*(tile_width-1), style=tile_style(range_color(mm)))
                lines.append('|\n')
            lines.append('  +'+'-'*((end-start)*tile_width)+'+')
            parts.append(lines)
        else:
            lines = [f'cols {start:02d}..{end-1:02d}', '  +'+'-'*(end-start)+'+']
            for r in range(8):
                cells = ''
                for v in values[r*64+start:r*64+end]:
                    if v == 0: cells += '-'
                    elif not intensity and not 100 <= v <= 5000: cells += '?'
                    else:
                        a, b = (low, high) if intensity else (100, 5000)
                        cells += '@%#*+=:.'[min(7, int((v-a)/max(1, b-a)*7))]
                lines.append(f'{r} |{cells}|')
            lines.append('  +'+'-'*(end-start)+'+')
            parts.append(Text('\n'.join(lines)))
    return Group(*parts)


def sensor(data, width, *, full=False, colored=False, cell=None):
    name = data.get('sensor', 'sensor')
    name = {'bump':'imu / impacts', 'intensity':'lidar / intensity', 'camera':'huskylens'}.get(name, name)
    age = max(0, time.monotonic()-data['_received_at']) if '_received_at' in data else data.get('age_seconds')
    meta = [name.upper()]
    if data.get('device'): meta.append(clean(data['device']))
    for key, label in (('frame_id', 'frame'), ('timestamp_ms', 'device ms')):
        if key in data: meta.append(f'{label} {data[key]}')
    if age is not None: meta.append(f'age {age:.3f}s '+('STALE' if age > .5 else 'fresh'))
    if 'state' in data: meta.append(str(data['state']))
    parts = [Text('  |  '.join(meta), style='bold')]
    used = {'test', 'sensor', 'device', 'frame_id', 'timestamp_ms', 'age_seconds', '_received_at', 'state', '_cell', '_full'}
    for key in ('distance_mm', 'intensity_raw'):
        if key in data:
            parts.append(grid(data[key], width, full=full, intensity=key=='intensity_raw', colored=colored, cell=cell))
            used.add(key)
    if 'accel_mg' in data:
        parts.append(table(['X mg', 'Y mg', 'Z mg'], [data['accel_mg']]))
        used.add('accel_mg')
    if 'field_ut' in data:
        parts.append(table(['HEADING deg', 'X uT', 'Y uT', 'Z uT'], [[data['heading_deg'], *data['field_ut']]]))
        used.update(('field_ut', 'heading_deg'))
    if 'direction' in data:
        direction = data['direction']
        parts.append(Text(f'impact {direction} '+('X-', 'X+', 'Y-', 'Y+', 'Z-', 'Z+')[direction]))
        used.add('direction')
    if 'blocks' in data or 'arrows' in data:
        blocks, arrows = data.get('blocks', []), data.get('arrows', [])
        parts.append(Text(f"{data.get('width', '?')}x{data.get('height', '?')} px  blocks {len(blocks)}  arrows {len(arrows)}"))
        if blocks:
            parts.append(table(['ID', 'CONF', 'X px', 'Y px', 'W px', 'H px'],
                               [[v[4], v[5], *v[:4]] for v in blocks], 'BLOCKS'))
        if arrows:
            parts.append(table(['ID', 'CONF', 'X0 px', 'Y0 px', 'X1 px', 'Y1 px'],
                               [[v[4], v[5], *v[:4]] for v in arrows], 'ARROWS'))
        if not blocks and not arrows: parts.append(Text('no detections'))
        used.update(('blocks', 'arrows', 'width', 'height'))
    extra = {k:v for k,v in data.items() if k not in used}
    if extra: parts.append(table(['FIELD', 'VALUE'], fields(extra)))
    return Group(*parts)


def render(data, width=80, *, full=True, colored=False, cell=None):
    if 'sensor' in data:
        return sensor(data, width, full=full, colored=colored, cell=cell if cell is not None else data.get('_cell'))
    title = data.get('test', 'K10').upper()
    parts, rest = [Text(title, style='bold')], {k:v for k,v in data.items() if k != 'test'}
    for key, unit in (('commanded_degrees', 'DEG (commanded)'), ('commanded_percent', '% (commanded)')):
        if key in rest:
            parts.append(table(['CHANNEL', unit], rest.pop(key).items()))
    if isinstance(rest.get('leds'), dict):
        parts.append(table(['LED', 'RED', 'GREEN', 'BLUE', 'BRIGHTNESS'],
            [[ch, v.get('red'), v.get('green'), v.get('blue'), v.get('brightness')] for ch,v in rest.pop('leds').items()]))
    if isinstance(rest.get('board_leds'), dict):
        board = dict(rest.pop('board_leds'))
        leds = board.pop('leds', [])
        if leds:
            headers = list(dict.fromkeys(k for led in leds for k in led))
            parts.append(table([k.upper() for k in headers], [[led.get(k) for k in headers] for led in leds], 'BOARD LEDS'))
        else: parts.append(Text('no board LED readings'))
        rest.update(board)
    if 'sounds' in rest and isinstance(rest['sounds'], list):
        sounds = rest.pop('sounds')
        parts.append(table(['FILE'], [[name] for name in sounds]) if sounds else Text('no stored sounds'))
    if rest: parts.append(table(['FIELD', 'VALUE'], fields(rest)))
    return Group(*parts)


def output(data):
    console = Console(highlight=False, markup=False, emoji=False)
    console.print(render(data, console.width, full=data.get('_full', False), colored=has_color(console)))


def has_color(console):
    return bool(getattr(console.file, 'isatty', lambda:False)() and console.is_terminal
                and console.color_system and not console.no_color)


class Watch:
    """Update this terminal only; append complete snapshots for pipes/overflow."""
    def __init__(self, *, full=False, console=None, cell=None):
        self.console = console or Console(highlight=False, markup=False, emoji=False)
        self.full, self.live = full, None
        self.cell = cell

    def __enter__(self):
        return self

    def update(self, records):
        content = Group(*[render(data, self.console.width, full=self.full, colored=has_color(self.console), cell=self.cell) for data in records.values()])
        fits = len(self.console.render_lines(content, pad=False)) < self.console.height
        tty = getattr(self.console.file, 'isatty', lambda:False)()
        if tty and self.console.is_terminal and not self.console.is_dumb_terminal and fits:
            if self.live is None:
                self.live = Live(content, console=self.console, screen=False, auto_refresh=False,
                                 redirect_stdout=False, redirect_stderr=False)
                self.live.start(refresh=True)
            else: self.live.update(content, refresh=True)
        else:
            self.close()
            self.console.print(content)

    def close(self):
        if self.live is not None:
            self.live.stop()
            self.live = None

    def __exit__(self, *exc):
        self.close()
