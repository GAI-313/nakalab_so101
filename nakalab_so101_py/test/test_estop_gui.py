"""Tests for the emergency stop GIF GUI viewer."""

import ctypes
from pathlib import Path
from types import SimpleNamespace

from nakalab_so101_py import estop_gui
from nakalab_so101_py.estop_gui import (
    can_access_display,
    default_emergency_gif_path,
    EstopGuiNode,
    EstopGuiViewer,
    load_gif_frames,
)

from rclpy.node import Node


class FakeCv2:
    """Fake OpenCV module recording GUI window operations."""

    WINDOW_AUTOSIZE = 1

    def __init__(self):
        """Initialize recorded OpenCV calls."""
        self.calls = []

    def namedWindow(self, name, flags):  # noqa: N802
        """Record window creation."""
        self.calls.append(('namedWindow', name, flags))

    def imshow(self, name, frame):
        """Record frame display."""
        self.calls.append(('imshow', name, frame.shape))

    def waitKey(self, delay):  # noqa: N802
        """Record GUI event pump."""
        self.calls.append(('waitKey', delay))
        return -1

    def destroyWindow(self, name):  # noqa: N802
        """Record window destruction."""
        self.calls.append(('destroyWindow', name))

    def destroyAllWindows(self):  # noqa: N802
        """Record fallback window cleanup."""
        self.calls.append(('destroyAllWindows',))


def _repo_gif_path() -> Path:
    return (
        Path(__file__).resolve().parents[2] /
        'nakalab_so101_description' /
        'video' /
        'emergency_stop.gif'
    )


def _patch_node(monkeypatch):
    class FakeLogger:
        """Collect warning and info logs."""

        def __init__(self):
            """Initialize log buffers."""
            self.warnings = []
            self.infos = []

        def warning(self, message):
            """Record a warning log."""
            self.warnings.append(message)

        def info(self, message):
            """Record an info log."""
            self.infos.append(message)

    def node_init(self, node_name, parameter_overrides=None):
        del node_name, parameter_overrides
        self.fake_logger = FakeLogger()
        self.fake_parameters = {}

    monkeypatch.setattr(Node, '__init__', node_init)
    monkeypatch.setattr(
        Node,
        'declare_parameter',
        lambda self, name, val: self.fake_parameters.setdefault(name, val),
    )
    monkeypatch.setattr(
        Node,
        'get_parameter',
        lambda self, name: SimpleNamespace(value=self.fake_parameters[name]),
    )
    monkeypatch.setattr(
        Node,
        'create_subscription',
        lambda self, *args, **kwargs: SimpleNamespace(),
    )
    monkeypatch.setattr(Node, 'get_logger', lambda self: self.fake_logger)


def test_default_emergency_gif_path_uses_ament_index(monkeypatch):
    """Resolve emergency_stop.gif via get_package_share_directory."""
    monkeypatch.setattr(
        estop_gui,
        'get_package_share_directory',
        lambda pkg: f'/fake/share/{pkg}',
    )

    path = default_emergency_gif_path()

    assert path == Path(
        '/fake/share/nakalab_so101_description/video/emergency_stop.gif'
    )


def test_can_access_display_checks_env_and_x11(monkeypatch):
    """Detect unset DISPLAY and unreachable X11 displays."""
    monkeypatch.delenv('DISPLAY', raising=False)
    assert can_access_display() is False

    monkeypatch.setenv('DISPLAY', ':99')
    closed = []

    class FakeX11:
        """Fake libX11 wrapper."""

        def __init__(self, display_ptr):
            """Initialize fake XOpenDisplay and XCloseDisplay."""
            self.display_ptr = display_ptr

            def open_display(_arg):
                return self.display_ptr

            def close_display(ptr):
                closed.append(ptr)

            self.XOpenDisplay = open_display
            self.XCloseDisplay = close_display

    monkeypatch.setattr(ctypes, 'CDLL', lambda _name: FakeX11(None))
    assert can_access_display() is False

    monkeypatch.setattr(ctypes, 'CDLL', lambda _name: FakeX11(1234))
    assert can_access_display() is True
    assert closed == [1234]


def test_load_gif_frames_reads_emergency_stop_animation():
    """Decode all frames and frame durations from emergency_stop.gif."""
    frames, durations = load_gif_frames(_repo_gif_path())

    assert len(frames) == 6
    assert frames[0].shape == (360, 480, 3)
    assert all(duration > 0.0 for duration in durations)


def test_viewer_opens_animates_and_closes_with_estop():
    """Open and animate while E-STOP is active, then close on recovery."""
    frames, durations = load_gif_frames(_repo_gif_path())
    fake_cv2 = FakeCv2()
    clock = {'now': 0.0}
    viewer = EstopGuiViewer(
        frames,
        durations,
        cv2_module=fake_cv2,
        monotonic_func=lambda: clock['now'],
    )

    viewer.update(False)
    assert fake_cv2.calls == []

    viewer.update(True)
    assert viewer.window_open is True
    assert viewer.frame_index == 0
    assert fake_cv2.calls[0][0] == 'namedWindow'
    assert fake_cv2.calls[1][0] == 'imshow'

    clock['now'] += durations[0] + 0.01
    viewer.update(True)
    assert viewer.frame_index == 1

    viewer.update(False)
    assert viewer.window_open is False
    assert ('destroyWindow', viewer.window_name) in fake_cv2.calls


def test_estop_gui_node_skips_display_when_inaccessible(monkeypatch):
    """Disable GUI rendering safely when display probe fails."""
    _patch_node(monkeypatch)
    monkeypatch.setattr(
        estop_gui,
        'default_emergency_gif_path',
        _repo_gif_path,
    )

    node = EstopGuiNode(display_checker=lambda: False)
    node.estop_callback(SimpleNamespace(data=True))
    node.step_gui()

    assert node.display_available is False
    assert node.viewer.window_open is False
    assert len(node.fake_logger.infos) == 1


def test_estop_gui_node_closes_window_when_power_restored(monkeypatch):
    """Close emergency stop GUI as soon as valid supply voltage returns."""
    _patch_node(monkeypatch)
    monkeypatch.setattr(
        estop_gui,
        'default_emergency_gif_path',
        _repo_gif_path,
    )

    node = EstopGuiNode(display_checker=lambda: True)
    fake_cv2 = FakeCv2()
    node.viewer.cv2 = fake_cv2

    # Power lost -> open window.
    node.estop_callback(SimpleNamespace(data=True))
    node.voltage_callback(SimpleNamespace(data=0.0))
    node.step_gui()
    assert node.viewer.window_open is True

    # E-STOP button released and normal voltage returns -> close window.
    node.voltage_callback(SimpleNamespace(data=12.0))
    node.step_gui()
    assert node.viewer.window_open is False

    # If voltage drops again before recovery, reopen window.
    node.voltage_callback(SimpleNamespace(data=0.0))
    node.step_gui()
    assert node.viewer.window_open is True

    # Recovery clears estop flag -> close window.
    node.estop_callback(SimpleNamespace(data=False))
    node.step_gui()
    assert node.viewer.window_open is False
