"""Shared distance colors and semantics; no physical sensor access."""
import subprocess
import sys

from botticelli.display import closest, range_color as color, distance, palette


def test_shared_palette_and_invalid_colors_are_distinct():
    assert len(palette()) == 256
    assert color(100) == palette()[0]
    assert color(5000) == palette()[-1]
    assert color(0) != color(99) != color(100)
    assert color(65535) == color(99)


def test_distances_never_present_invalid_returns_as_real_ranges():
    assert distance(400) == '0.400 m (400 mm)'
    assert distance(0) == 'NO RETURN (0 mm)'
    assert distance(99) == 'OUT OF RANGE (99 mm)'
    assert distance(65535) == 'OUT OF RANGE (65535 mm)'
    assert closest([0,99,400,5000,65535]) == (400,2)
    assert closest([0,99,65535]) is None


def test_palette_obeys_configured_display_range():
    assert color(1000,1000,2000) == palette()[0]
    assert color(2000,1000,2000) == palette()[-1]
    assert 'OUT OF RANGE' in distance(400,1000,2000)


def test_terminal_heatmap_needs_no_simulation_or_plotting_packages():
    result = subprocess.run([sys.executable, '-c', '''
import sys
from botticelli.display import palette, render
render({'sensor': 'lidar', 'distance_mm': [400]*512}, colored=True)
assert len(palette()) == 256
assert not {'matplotlib', 'numpy', 'pygame', 'irsim'} & sys.modules.keys()
'''], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
