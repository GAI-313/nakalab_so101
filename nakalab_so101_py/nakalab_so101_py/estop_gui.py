"""Display an animated emergency stop GIF when E-STOP is active."""

import ctypes
import os
from pathlib import Path
import time
from typing import Callable, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from PIL import Image, ImageSequence
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


DEFAULT_WINDOW_NAME = 'SO-101 EMERGENCY STOP'
DEFAULT_FRAME_DURATION_SEC = 0.07


def default_emergency_gif_path() -> Path:
    """Return the installed emergency_stop.gif path via ament_index."""
    package_share = Path(
        get_package_share_directory('nakalab_so101_description')
    )
    return package_share / 'video' / 'emergency_stop.gif'


def can_access_display() -> bool:
    """Return whether an X11 display is currently accessible."""
    if not os.environ.get('DISPLAY'):
        return False

    try:
        x11 = ctypes.CDLL('libX11.so.6')
    except OSError:
        return True

    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x11.XOpenDisplay(None)
    if not display:
        return False
    x11.XCloseDisplay(display)
    return True


def load_gif_frames(gif_path: Path) -> Tuple[List[np.ndarray], List[float]]:
    """Load BGR frames and per-frame durations from an animated GIF."""
    path = Path(gif_path)
    if not path.is_file():
        raise FileNotFoundError(f'Emergency stop GIF not found: {path}')

    frames: List[np.ndarray] = []
    durations: List[float] = []

    with Image.open(path) as image:
        for frame in ImageSequence.Iterator(image):
            rgb_frame = frame.convert('RGB')
            bgr_frame = np.array(rgb_frame)[:, :, ::-1].copy()
            duration_ms = frame.info.get('duration', 0)
            duration_sec = (
                float(duration_ms) / 1000.0
                if duration_ms and duration_ms > 0
                else DEFAULT_FRAME_DURATION_SEC
            )
            frames.append(bgr_frame)
            durations.append(duration_sec)

    if not frames:
        raise ValueError(f'No frames decoded from GIF: {path}')

    return frames, durations


class EstopGuiViewer:
    """Manage OpenCV window rendering for the emergency stop animation."""

    def __init__(
        self,
        frames: List[np.ndarray],
        durations: List[float],
        window_name: str = DEFAULT_WINDOW_NAME,
        cv2_module=cv2,
        monotonic_func=time.monotonic,
    ):
        """Initialize frame buffer and display state."""
        self.frames = frames
        self.durations = durations
        self.window_name = window_name
        self.cv2 = cv2_module
        self.monotonic = monotonic_func
        self.window_open = False
        self.frame_index = 0
        self.last_frame_time = 0.0

    def update(self, estop_active: bool) -> None:
        """Show/animate the window while estop_active, otherwise close it."""
        if not estop_active:
            self.close()
            return

        now = self.monotonic()
        if not self.window_open:
            self.cv2.namedWindow(self.window_name, self.cv2.WINDOW_AUTOSIZE)
            self.window_open = True
            self.frame_index = 0
            self.last_frame_time = now
            self.cv2.imshow(self.window_name, self.frames[self.frame_index])
            self.cv2.waitKey(1)
            return

        if now - self.last_frame_time >= self.durations[self.frame_index]:
            self.frame_index = (self.frame_index + 1) % len(self.frames)
            self.last_frame_time = now
            self.cv2.imshow(self.window_name, self.frames[self.frame_index])

        self.cv2.waitKey(1)

    def close(self) -> None:
        """Close the OpenCV window if it is open."""
        if not self.window_open:
            return
        try:
            self.cv2.destroyWindow(self.window_name)
        except Exception:
            self.cv2.destroyAllWindows()
        for _ in range(5):
            self.cv2.waitKey(1)
        self.window_open = False


class EstopGuiNode(Node):
    """Subscribe to follower E-STOP and voltage and drive the GUI viewer."""

    def __init__(
        self,
        parameter_overrides: Optional[list] = None,
        display_checker: Callable[[], bool] = can_access_display,
    ):
        """Initialize subscriptions and load the emergency stop GIF."""
        super().__init__(
            'estop_gui_node',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('estop_topic', 'follower/estop')
        self.declare_parameter('voltage_topic', 'follower/voltage')
        self.declare_parameter('min_voltage', 5.5)
        self.declare_parameter('gif_path', str(default_emergency_gif_path()))

        estop_topic = self.get_parameter('estop_topic').value
        voltage_topic = self.get_parameter('voltage_topic').value
        self.min_voltage = float(self.get_parameter('min_voltage').value)
        gif_path = Path(self.get_parameter('gif_path').value)

        self.estop_active = False
        self.latest_voltage: Optional[float] = None
        self.display_available = display_checker()
        frames, durations = load_gif_frames(gif_path)
        self.viewer = EstopGuiViewer(frames, durations)

        if not self.display_available:
            self.get_logger().info(
                'Display is not accessible; emergency stop GUI disabled.'
            )
            return

        self.estop_sub = self.create_subscription(
            Bool,
            estop_topic,
            self.estop_callback,
            10,
        )
        self.voltage_sub = self.create_subscription(
            Float32,
            voltage_topic,
            self.voltage_callback,
            10,
        )
        self.get_logger().info(
            f'E-STOP GUI watching {estop_topic} and {voltage_topic} '
            f'with {gif_path}'
        )

    def estop_callback(self, msg: Bool) -> None:
        """Update the latest E-STOP status from ROS."""
        self.estop_active = bool(msg.data)

    def voltage_callback(self, msg: Float32) -> None:
        """Update the latest measured servo supply voltage."""
        self.latest_voltage = float(msg.data)

    def should_show_gui(self) -> bool:
        """Return whether the emergency stop animation should be visible."""
        if not self.estop_active:
            return False
        if self.min_voltage > 0.0 and self.latest_voltage is not None:
            return self.latest_voltage < self.min_voltage
        return True

    def step_gui(self) -> None:
        """Update the GUI window if a display server is available."""
        if not self.display_available:
            return
        try:
            self.viewer.update(self.should_show_gui())
        except cv2.error as exc:
            self.get_logger().warning(
                f'Disabling emergency stop GUI after OpenCV error: {exc}'
            )
            self.display_available = False


def main(args=None):
    """Run the emergency stop GUI ROS2 node."""
    rclpy.init(args=args)
    node = EstopGuiNode()
    try:
        if node.display_available:
            while rclpy.ok() and node.display_available:
                rclpy.spin_once(node, timeout_sec=0.01)
                node.step_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
