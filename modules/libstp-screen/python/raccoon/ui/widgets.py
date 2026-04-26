"""
UI Widgets - Type-safe widget definitions for dynamic screens.

These dataclasses serialize to JSON and are rendered by Flutter.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# ============================================================
# BASE WIDGET
# ============================================================


@dataclass
class Widget:
    """Base class for all widgets.

    Concrete on purpose — ``to_dict`` works for any subclass that adds
    plain dataclass fields, and the base never had an abstract method
    that would have justified inheriting from ``ABC``.
    """

    def to_dict(self) -> dict:
        """Convert to JSON-serializable dict."""
        result = {"widget": self.__class__.__name__}
        for key, value in self.__dict__.items():
            if key.startswith("_"):
                continue
            if isinstance(value, Widget):
                result[key] = value.to_dict()
            elif isinstance(value, list):
                result[key] = [
                    item.to_dict() if isinstance(item, Widget) else item for item in value
                ]
            elif isinstance(value, tuple):
                result[key] = list(value)
            else:
                result[key] = value
        return result


# ============================================================
# DISPLAY WIDGETS
# ============================================================


@dataclass
class Text(Widget):
    """Text display widget."""

    text: str
    size: str = "medium"  # small, medium, large, title
    color: str | None = None  # hex color or named color
    bold: bool = False
    muted: bool = False  # grey/subtle text
    align: str = "left"  # left, center, right


@dataclass
class Icon(Widget):
    """Material icon widget."""

    name: str  # Material icon name (e.g., "check", "warning", "touch_app")
    size: int = 24
    color: str | None = None


@dataclass
class Spacer(Widget):
    """Vertical spacing widget."""

    height: int = 16


@dataclass
class Divider(Widget):
    """Horizontal divider line."""

    thickness: int = 1
    color: str | None = None


@dataclass
class StatusBadge(Widget):
    """Colored pill/badge with text (e.g., "LIGHT ON")."""

    text: str
    color: str = "grey"  # grey, green, amber, red, blue, orange
    glow: bool = False  # Add glow effect


@dataclass
class StatusIcon(Widget):
    """Animated circle with icon inside."""

    icon: str  # check, warning, error, info
    color: str = "green"  # green, orange, red, blue
    animated: bool = True


@dataclass
class HintBox(Widget):
    """Highlighted hint box (e.g., "Press button when ready")."""

    text: str
    icon: str = "touch_app"
    style: str = "normal"  # normal, prominent


@dataclass
class DistanceBadge(Widget):
    """Distance value display badge."""

    value: float
    unit: str = "cm"
    color: str = "blue"


@dataclass
class ResultsTable(Widget):
    """Key-value results table."""

    # List of (label, value, color?) tuples
    rows: list[tuple] = field(default_factory=list)


# ============================================================
# INPUT WIDGETS
# ============================================================


@dataclass
class Button(Widget):
    """Clickable button."""

    id: str
    label: str
    style: str = "primary"  # primary, secondary, success, danger, warning
    icon: str | None = None
    disabled: bool = False


@dataclass
class Slider(Widget):
    """Slider input for numeric values."""

    id: str
    min: float
    max: float
    value: float = 0
    label: str | None = None
    show_value: bool = True


@dataclass
class Checkbox(Widget):
    """Checkbox toggle."""

    id: str
    label: str
    value: bool = False


@dataclass
class Dropdown(Widget):
    """Dropdown selection."""

    id: str
    options: list[str] = field(default_factory=list)
    value: str | None = None
    label: str | None = None


@dataclass
class NumericKeypad(Widget):
    """Touch-friendly numeric keypad (0-9, ., backspace)."""


@dataclass
class NumericInput(Widget):
    """Large numeric display with optional +/- buttons."""

    id: str
    value: float = 0
    unit: str = ""
    min_value: float | None = None
    max_value: float | None = None
    show_adjust_buttons: bool = True


@dataclass
class TextInput(Widget):
    """Text input field."""

    id: str
    value: str = ""
    label: str | None = None
    placeholder: str = ""


# ============================================================
# VISUALIZATION WIDGETS (Pre-built in Flutter)
# ============================================================


@dataclass
class SensorValue(Widget):
    """Large sensor value display."""

    port: int
    sensor_type: str = "analog"  # analog, digital


@dataclass
class SensorGraph(Widget):
    """Real-time sensor line graph."""

    port: int
    sensor_type: str = "analog"
    max_points: int = 50


@dataclass
class LightBulb(Widget):
    """Animated light bulb visualization."""

    is_on: bool = False


@dataclass
class AnimatedRobot(Widget):
    """Robot with spinning wheels visualization."""

    moving: bool = False
    size: int = 120


@dataclass
class CircularSlider(Widget):
    """Circular slider for motor/servo control."""

    id: str
    min: float
    max: float
    value: float = 0
    label: str | None = None


@dataclass
class ProgressSpinner(Widget):
    """Circular loading spinner."""

    size: int = 24
    color: str | None = None


@dataclass
class PulsingArrow(Widget):
    """Animated pulsing arrow."""

    direction: str = "right"  # right, left, up, down


@dataclass
class RobotDrivingAnimation(Widget):
    """Robot driving on track animation."""

    target_distance: float = 30.0


@dataclass
class MeasuringTape(Widget):
    """Animated measuring tape."""

    distance: float = 30.0


@dataclass
class CalibrationChart(Widget):
    """
    Static scatter/line chart for calibration data.

    Shows collected sample points with horizontal threshold lines.
    """

    samples: list[float] = field(default_factory=list)
    thresholds: list[tuple] = field(default_factory=list)  # [(value, label, color), ...]
    height: int = 200


# ============================================================
# LAYOUT WIDGETS
# ============================================================


@dataclass
class Row(Widget):
    """Horizontal layout - children side by side."""

    children: list[Widget] = field(default_factory=list)
    align: str = "center"  # start, center, end, space_between, space_around
    spacing: int = 8


@dataclass
class Column(Widget):
    """Vertical layout - children stacked."""

    children: list[Widget] = field(default_factory=list)
    align: str = "stretch"  # start, center, end, stretch
    spacing: int = 12


@dataclass
class Center(Widget):
    """Center children vertically and horizontally."""

    children: list[Widget] = field(default_factory=list)


@dataclass
class Container(Widget):
    """Container with optional background color, fills available space."""

    children: list[Widget] = field(default_factory=list)
    bg_color: str | None = None  # hex color or named color
    padding: int = 0


@dataclass
class Card(Widget):
    """Card container with optional title."""

    children: list[Widget] = field(default_factory=list)
    title: str | None = None
    padding: int = 16


@dataclass
class Split(Widget):
    """Left-right split layout."""

    left: list[Widget] = field(default_factory=list)
    right: list[Widget] = field(default_factory=list)
    ratio: tuple = (1, 1)  # e.g., (5, 3) = left gets 5 parts, right gets 3


@dataclass
class Expanded(Widget):
    """Expand child to fill available space."""

    child: Widget = None
    flex: int = 1
