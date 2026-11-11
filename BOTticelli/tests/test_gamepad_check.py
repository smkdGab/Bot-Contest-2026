from botticelli.dashboard import Checklist
from botticelli.dashboard import CheckWindow
from botticelli.config import Gamepad
from botticelli.control import Input
from types import SimpleNamespace
import pytest


def test_checklist_tracks_full_button_cycles_axes_and_dpad():
    check = Checklist()
    check.update([False], [0], [(0, 0)])
    assert check.buttons == [[False, False]]
    check.update([True], [-1], [(1, 1)])
    assert check.buttons == [[True, False]]
    check.update([False], [1], [(-1, -1)])
    assert check.buttons == [[True, True]]
    assert check.axes == [[True, True]]
    assert len(check.hats[0]) == 4


@pytest.mark.parametrize("dpad", [False, True])
def test_view_shows_only_named_driving_controls(dpad, monkeypatch):
    window = CheckWindow(Gamepad(dpad=dpad))
    labels = []
    monkeypatch.setattr(window, "text", lambda value, *args, **kwargs: labels.append(value))
    try:
        reader = SimpleNamespace(message="Test controller")
        window.draw(reader, Input(0, True, throttle=1, steering=-1, a=True))
        window.draw(reader, Input(0, True, throttle=-1, steering=1))
        assert "Left stick" in labels
        assert ("D-pad" in labels) == dpad
        assert any("[OK] A - hold to drive" in label for label in labels)
        assert all(all(history) for history in window.history.axes)
        assert not any("Button " in label or "Axis " in label or "Hat " in label for label in labels)
    finally:
        window.close()
