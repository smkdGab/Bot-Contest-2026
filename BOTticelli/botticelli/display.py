"""Terminal presentation only: Rich tables/live output and ASCII sensor grids."""
import time

from rich import box
from rich.console import Console, Group
from rich.live import Live
from rich.table import Table
from rich.text import Text


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


def grid(values, width, *, full, intensity=False):
    """Preserve wire row/column order; split columns to fit narrow terminals."""
    valid = [v for v in values if v]
    low, high = (min(valid), max(valid)) if valid else (0, 0)
    unit = 'raw' if intensity else 'mm'
    summary = f'8x64 {unit}  nonzero {len(valid)}/512  min {low}  max {high}'
    summary += f'  mean {sum(valid)/len(valid):.1f}' if valid else '  mean -'
    parts = [Text(summary)]
    cell_width = max(3, max(len(str(v)) for v in values)) if full else 1
    columns = min(64, max(1, (width-6)//(cell_width+1))) if full else min(64, max(1, width-6))
    if not full:
        parts.append(Text(f'scale {low}..{high} raw' if intensity else 'scale 100..5000 mm; @ near, . far'))
        parts.append(Text('- zero; --full shows exact values' if intensity else
                          '- no return; ? outside nominal range; --full shows exact values'))
    for start in range(0, 64, columns):
        end = min(64, start+columns)
        if full:
            parts.append(table(['r/c']+[str(c) for c in range(start, end)],
                [[r]+list(values[r*64+start:r*64+end]) for r in range(8)], grid=True))
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


def sensor(data, width, *, full=False):
    name = data.get('sensor', 'sensor')
    age = max(0, time.monotonic()-data['_received_at']) if '_received_at' in data else data.get('age_seconds')
    meta = [name.upper()]
    for key, label in (('frame_id', 'frame'), ('timestamp_ms', 'device ms')):
        if key in data: meta.append(f'{label} {data[key]}')
    if age is not None: meta.append(f'age {age:.3f}s '+('STALE' if age > .5 else 'fresh'))
    if 'state' in data: meta.append(str(data['state']))
    parts = [Text('  |  '.join(meta), style='bold')]
    used = {'test', 'sensor', 'frame_id', 'timestamp_ms', 'age_seconds', '_received_at', 'state'}
    for key in ('distance_mm', 'intensity_raw'):
        if key in data:
            parts.append(grid(data[key], width, full=full, intensity=key=='intensity_raw'))
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


def render(data, width=80, *, full=True):
    if 'sensor' in data:
        return sensor(data, width, full=full)
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
    console.print(render(data, console.width))


class Watch:
    """Update this terminal only; append complete snapshots for pipes/overflow."""
    def __init__(self, *, full=False, console=None):
        self.console = console or Console(highlight=False, markup=False, emoji=False)
        self.full, self.live = full, None

    def __enter__(self):
        return self

    def update(self, records):
        content = Group(*[render(data, self.console.width, full=self.full) for data in records.values()])
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
