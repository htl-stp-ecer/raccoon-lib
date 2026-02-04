# BotUI Protocol v2 - High-Level Design

A high-level, annotation-based system for creating native-feeling robot UI screens with minimal code.

## Design Philosophy

1. **Opinionated over flexible** - Pre-built patterns that look great out of the box
2. **Complex widgets are first-class** - Graphs, keypads, circular sliders, custom visualizations
3. **Annotation-based feedback** - Decorators bind methods to UI events
4. **State machines built-in** - Multi-step flows are common, make them easy
5. **Python does logic, Flutter renders** - Keep it simple

---

## Python API Design

### 1. Simple Dialogs (One-shot screens)

```python
from libstp.ui import *

# Simplest: Just show a message and wait for confirmation
@dsl(tags=["ui"])
class WaitForSetup(UiScreen):
    """Shows instruction and waits for button press."""

    title = "Setup Required"

    # Declarative layout - no build_screen() needed for simple cases
    layout = CenteredLayout([
        Icon("warning", color="amber", size=64),
        Text("Place robot at starting position", size="large"),
        Spacer(32),
        HintBox("Press the button when ready", icon="touch_app"),
    ])

    # Annotation-based feedback - method called when button pressed
    @on_button_press
    async def on_ready(self, robot):
        self.info("User pressed button, continuing...")


# Factory for DSL
@dsl(name="Wait for Setup", tags=["ui", "setup"])
def wait_for_setup() -> WaitForSetup:
    return WaitForSetup()
```

### 2. Confirmation Dialogs

```python
@dsl(tags=["ui", "dialog"])
class ConfirmAction(UiScreen):
    """Two-button confirmation dialog."""

    def __init__(self, title: str, message: str):
        super().__init__()
        self.title = title
        self.message = message
        self.confirmed = False

    layout = CenteredLayout([
        Icon("help_outline", color="blue", size=48),
        Text(lambda self: self.message, size="large"),
        Spacer(24),
        ButtonRow([
            Button("no", "Cancel", style="secondary"),
            Button("yes", "Confirm", style="success"),
        ]),
    ])

    @on_click("yes")
    async def on_confirm(self, robot):
        self.confirmed = True

    @on_click("no")
    async def on_cancel(self, robot):
        self.confirmed = False


@dsl(tags=["ui", "dialog"])
def confirm(title: str = "Confirm", message: str = "Continue?") -> ConfirmAction:
    return ConfirmAction(title, message)
```

### 3. Input Dialogs

```python
@dsl(tags=["ui", "input"])
class NumberInput(UiScreen):
    """Numeric input with keypad."""

    def __init__(self, title: str, label: str, default: float = 0, unit: str = ""):
        super().__init__()
        self.title = title
        self.label = label
        self.unit = unit
        self.value = default

    layout = SplitLayout(
        left=[
            Text(lambda self: self.label, size="title"),
            Spacer(16),
            # NumericDisplay is a pre-built widget showing the current value
            NumericDisplay(
                value=lambda self: self.value,
                unit=lambda self: self.unit,
                with_adjust_buttons=True,  # +/- buttons
            ),
            Spacer(16),
            Button("submit", "Submit", style="success", icon="check"),
        ],
        right=[
            NumericKeypad(),  # Pre-built touch keypad
        ],
    )

    @on_keypad_input
    def on_key(self, key: str):
        if key == "back":
            self.value = float(str(self.value)[:-1] or "0")
        elif key == ".":
            if "." not in str(self.value):
                self.value = float(f"{self.value}.")
        else:
            self.value = float(f"{self.value}{key}")

    @on_adjust(delta=0.5)
    def on_adjust(self, delta: float):
        self.value = max(0, self.value + delta)

    @on_click("submit")
    async def on_submit(self, robot):
        pass  # Step completes, self.value is available


@dsl(tags=["ui", "input"])
def number_input(title: str, label: str, default: float = 0, unit: str = "") -> NumberInput:
    return NumberInput(title, label, default, unit)
```

### 4. Multi-State Screens (State Machine)

```python
@dsl(tags=["ui", "calibration"])
class WaitForLightCalibration(UiStateMachine):
    """Multi-step calibration with state transitions."""

    def __init__(self, sensor_port: int):
        super().__init__()
        self.sensor_port = sensor_port
        self.light_off = 0.0
        self.light_on = 0.0

    # Define states with their layouts
    states = {
        "measure_off": State(
            layout=SplitLayout(
                left=[
                    StatusBadge("LIGHT OFF", color="grey"),
                    LightBulb(is_on=False),  # Pre-built animated widget
                    Text("Turn OFF the lamp", size="large"),
                    HintBox("Press button when ready"),
                ],
                right=[
                    SensorDisplay(
                        port=lambda self: self.sensor_port,
                        with_graph=True,  # Real-time graph
                    ),
                ],
            ),
        ),

        "measure_on": State(
            layout=SplitLayout(
                left=[
                    StatusBadge("LIGHT ON", color="amber", glow=True),
                    LightBulb(is_on=True),
                    Text("Turn ON the lamp", size="large"),
                    HintBox("Press button when ready"),
                ],
                right=[
                    SensorDisplay(port=lambda self: self.sensor_port, with_graph=True),
                ],
            ),
        ),

        "confirm": State(
            layout=SplitLayout(
                left=[
                    StatusIcon(
                        icon=lambda self: "check" if self.is_good else "warning",
                        color=lambda self: "green" if self.is_good else "orange",
                    ),
                    Text(
                        lambda self: "Calibration Complete" if self.is_good else "Low Contrast",
                        size="large",
                    ),
                    Spacer(16),
                    ValueInputRow([
                        ValueInput("light_off", "Light OFF", icon="lightbulb_outline"),
                        ValueInput("light_on", "Light ON", icon="lightbulb", glow=True),
                    ]),
                ],
                right=[
                    StatsPanel([
                        Stat("Threshold", lambda self: self.threshold, color="blue"),
                        Stat("Difference", lambda self: self.difference,
                             color=lambda self: "green" if self.is_good else "orange"),
                    ]),
                    Spacer(16),
                    ButtonColumn([
                        Button("retry", "Retry", style="secondary"),
                        Button("confirm", "Confirm",
                               style=lambda self: "success" if self.is_good else "warning"),
                    ]),
                ],
            ),
        ),
    }

    initial_state = "measure_off"

    @property
    def threshold(self) -> float:
        return (self.light_off + self.light_on) / 2

    @property
    def difference(self) -> float:
        return abs(self.light_on - self.light_off)

    @property
    def is_good(self) -> bool:
        return self.difference > 100

    # State transitions triggered by events
    @on_button_press
    @in_state("measure_off")
    async def measure_off_value(self, robot):
        self.light_off = await self.read_sensor(self.sensor_port)
        self.transition_to("measure_on")

    @on_button_press
    @in_state("measure_on")
    async def measure_on_value(self, robot):
        self.light_on = await self.read_sensor(self.sensor_port)
        self.transition_to("confirm")

    @on_click("retry")
    @in_state("confirm")
    async def retry(self, robot):
        self.transition_to("measure_off")

    @on_click("confirm")
    @in_state("confirm")
    async def finish(self, robot):
        # Step completes when we return from final state handler
        pass


@dsl(tags=["calibration"])
def calibrate_wait_for_light(port: int) -> WaitForLightCalibration:
    return WaitForLightCalibration(port)
```

### 5. Distance Calibration (Complex Example)

```python
@dsl(tags=["ui", "calibration"])
class DistanceCalibration(UiStateMachine):
    """Full distance calibration flow with animations."""

    def __init__(self, distance_cm: float = 30.0):
        super().__init__()
        self.requested_distance = distance_cm
        self.measured_distance = 0.0

    states = {
        "prepare": State(
            layout=CenteredLayout([
                Row([
                    AnimatedRobot(moving=False),  # Pre-built robot visualization
                    PulsingArrow(),  # Pre-built animated arrow
                ]),
                Spacer(24),
                Text("Distance Calibration", size="title"),
                DistanceBadge(lambda self: self.requested_distance),
                Text("Robot will drive this distance forward", muted=True),
                Spacer(24),
                HintBox("Press button to start", style="prominent"),
            ]),
        ),

        "driving": State(
            layout=CenteredLayout([
                RobotDrivingAnimation(  # Pre-built: robot on track with motion lines
                    target_distance=lambda self: self.requested_distance,
                ),
                Spacer(24),
                Row([
                    Spinner(),
                    Text("Robot is driving...", size="large"),
                ]),
                DistanceBadge(lambda self: self.requested_distance, color="orange"),
            ]),
        ),

        "measure": State(
            layout=SplitLayout(
                left=[
                    Text("Enter actual distance", size="title"),
                    Text(
                        lambda self: f"Robot attempted: {self.requested_distance:.0f} cm",
                        muted=True,
                    ),
                    Spacer(16),
                    NumericDisplay(
                        value=lambda self: self.measured_distance,
                        unit="cm",
                        with_adjust_buttons=True,
                    ),
                    Spacer(16),
                    Button("submit", "Submit", style="success", icon="check"),
                ],
                right=[
                    NumericKeypad(),
                ],
            ),
        ),

        "confirm": State(
            layout=SplitLayout(
                left=[
                    Row([
                        StatusIcon(
                            icon=lambda self: "check" if self.is_good else "warning",
                            color=lambda self: "green" if self.is_good else "orange",
                            animated=True,
                        ),
                        Text(
                            lambda self: "Calibration Complete!" if self.is_good else "Large Adjustment",
                            size="large",
                        ),
                    ]),
                    Spacer(16),
                    ResultsTable([
                        ("Requested", lambda self: f"{self.requested_distance:.1f} cm"),
                        ("Measured", lambda self: f"{self.measured_distance:.1f} cm"),
                        ("Scale", lambda self: f"{self.scale_factor:.4f}", "blue"),
                        ("Adjust", lambda self: f"{self.adjustment:+.1f}%",
                         lambda self: "green" if self.is_good else "orange"),
                    ]),
                ],
                right=[
                    ButtonColumn([
                        Button("apply", "Apply",
                               style=lambda self: "success" if self.is_good else "warning",
                               icon="check"),
                        Button("retry", "Retry", style="secondary", icon="refresh"),
                    ]),
                ],
            ),
        ),
    }

    initial_state = "prepare"

    @property
    def scale_factor(self) -> float:
        if self.requested_distance == 0:
            return 1.0
        return self.measured_distance / self.requested_distance

    @property
    def adjustment(self) -> float:
        return (self.scale_factor - 1.0) * 100

    @property
    def is_good(self) -> bool:
        return abs(self.adjustment) < 10

    @on_button_press
    @in_state("prepare")
    async def start_driving(self, robot):
        self.transition_to("driving")
        # Actually drive the robot
        await robot.drive.forward(self.requested_distance)
        self.transition_to("measure")

    @on_keypad_input
    @in_state("measure")
    def on_key(self, key: str):
        # Update measured_distance based on keypad input
        ...

    @on_click("submit")
    @in_state("measure")
    async def submit_measurement(self, robot):
        self.transition_to("confirm")

    @on_click("retry")
    @in_state("confirm")
    async def retry(self, robot):
        self.measured_distance = 0.0
        self.transition_to("prepare")

    @on_click("apply")
    @in_state("confirm")
    async def apply(self, robot):
        # Save calibration
        robot.drive.set_scale_factor(self.scale_factor)
```

---

## Pre-Built Complex Widgets

These are **hardcoded Flutter implementations** that Python just references by name:

### Visualization Widgets

| Widget | Description | Properties |
|--------|-------------|------------|
| `LightBulb(is_on)` | Animated light bulb with glow | `is_on: bool` |
| `AnimatedRobot(moving)` | Robot with spinning wheels | `moving: bool, size: int` |
| `SensorGraph(port, type)` | Real-time line graph | `port: int, max_points: int` |
| `SensorDisplay(port)` | Big number + optional graph | `port: int, with_graph: bool` |
| `QuaternionViz()` | 3D orientation display | `show_legend: bool` |
| `CircularSlider(min, max)` | Motor/servo control | `min, max, value, label` |
| `MotorControl(port)` | Full motor control UI | `port: int` |
| `ServoControl(port)` | Full servo control UI | `port: int, angle_mode: bool` |

### Animation Widgets

| Widget | Description |
|--------|-------------|
| `PulsingArrow()` | Animated direction arrow |
| `RobotDrivingAnimation(distance)` | Robot moving on track |
| `MeasuringTape(distance)` | Animated tape unroll |
| `MotionLines()` | Speed lines behind robot |
| `Spinner()` | Circular progress |

### Input Widgets

| Widget | Description |
|--------|-------------|
| `NumericKeypad()` | 0-9, ., backspace touch keypad |
| `NumericDisplay(value, unit)` | Big number with +/- buttons |
| `ValueInput(id, label)` | Labeled editable value |

### Layout Widgets

| Widget | Description |
|--------|-------------|
| `StatusBadge(text, color)` | Colored status pill |
| `StatusIcon(icon, color)` | Animated status circle |
| `HintBox(text, icon)` | "Press button" style hint |
| `DistanceBadge(value)` | Distance display pill |
| `StatsPanel(stats)` | Grid of stat values |
| `ResultsTable(rows)` | Label-value table |

---

## Event Decorators

```python
# Button/hardware events
@on_button_press              # Physical button on robot
@on_screen_tap                # Tap anywhere on screen

# UI element events
@on_click("button_id")        # Specific button clicked
@on_submit                    # Form/input submitted
@on_cancel                    # Cancel/back pressed

# Input events
@on_keypad_input              # Keypad key pressed (receives key: str)
@on_slider_change("id")       # Slider value changed (receives value: float)
@on_value_change("id")        # Any input value changed

# State machine
@in_state("state_name")       # Only active in this state
@on_enter("state_name")       # Called when entering state
@on_exit("state_name")        # Called when leaving state
```

---

## Layout Types

### `CenteredLayout`
Everything centered vertically and horizontally. Good for simple dialogs.

```python
CenteredLayout([
    Icon("check", color="green"),
    Text("Success!", size="title"),
    Button("ok", "OK"),
])
```

### `SplitLayout`
Left-right split (like calibration screens). Default is 50/50.

```python
SplitLayout(
    left=[...],
    right=[...],
    ratio=(5, 3),  # Left gets 5 parts, right gets 3
)
```

### `Row` / `Column`
Horizontal / vertical stacking with optional alignment.

```python
Row([widget1, widget2], align="space_between")
Column([widget1, widget2], align="center")
```

### `ButtonRow` / `ButtonColumn`
Specialized for button groups with proper spacing.

---

## JSON Protocol

The Python side sends this structure:

```json
{
  "screen_type": "state_machine",
  "current_state": "measure_off",
  "title": "Wait for Light Calibration",
  "context": {
    "sensor_port": 3,
    "light_off": 1234.0,
    "light_on": 3456.0
  },
  "state_layout": {
    "type": "split",
    "ratio": [4, 3],
    "left": [
      {"widget": "status_badge", "text": "LIGHT OFF", "color": "grey"},
      {"widget": "light_bulb", "is_on": false},
      {"widget": "text", "text": "Turn OFF the lamp", "size": "large"},
      {"widget": "hint_box", "text": "Press button when ready"}
    ],
    "right": [
      {"widget": "sensor_display", "port": 3, "with_graph": true}
    ]
  }
}
```

### Response

```json
{
  "event": "button_press",
  "state": "measure_off"
}
```

or

```json
{
  "event": "click",
  "button_id": "confirm",
  "state": "confirm",
  "values": {
    "light_off": 1200,
    "light_on": 3500
  }
}
```

---

## Keeping Existing Screens

Your current hardcoded screens (BlackWhite calibration, WFL calibration, Distance calibration) should **stay as they are**. They're polished and work well.

The dynamic UI system is for **new screens** that users create. When Python sends a `screen_type` the Flutter side doesn't recognize, it uses the dynamic renderer. Known screen types route to hardcoded implementations.

```dart
// In screen_renderer_provider.dart
if (parsed['type'] == 'IR') {
  await handleBlackWhite(parsed);  // Existing hardcoded
} else if (parsed['type'] == 'waitForLight') {
  await handleWaitForLight(parsed);  // Existing hardcoded
} else if (parsed['type'] == 'dynamic_screen' || parsed['type'] == 'state_machine') {
  await handleDynamicScreen(parsed);  // New dynamic renderer
}
```

---

## Usage in Missions

```python
from libstp import *
from libstp.ui import *

class MyMission(Mission):
    def sequence(self) -> Step:
        # Pre-built UI step
        config = configure_speed(default=75)

        return Sequential([
            # Simple wait
            wait_for_setup(),

            # Configuration with values accessible later
            config,

            # Confirmation before dangerous action
            confirm("Warning", "Robot will move. Continue?"),

            # Use configured value
            drive_forward(100, speed=config.speed),

            # Number input
            number_input("Distance", "How far to go?", unit="cm"),
        ])
```

---

## Implementation Priority

### Phase 1: Core Infrastructure
1. `UiScreen` base class with event decorators
2. Simple layouts: `CenteredLayout`, `Row`, `Column`
3. Basic widgets: `Text`, `Button`, `Icon`, `Spacer`, `HintBox`
4. Flutter decoder for basic widgets

### Phase 2: Input Widgets
1. `NumericKeypad` + `NumericDisplay`
2. `@on_keypad_input` decorator
3. `ValueInput` for editable fields

### Phase 3: State Machine
1. `UiStateMachine` base class
2. State definitions with layouts
3. `@in_state`, `@on_enter`, `@on_exit` decorators
4. `transition_to()` method

### Phase 4: Complex Visualizations
1. Port existing painters to be reusable
2. `SensorDisplay`, `SensorGraph`
3. `LightBulb`, `AnimatedRobot`
4. `CircularSlider` integration

### Phase 5: Sensor Integration
1. Real-time sensor data in widgets
2. `SensorDisplay(port)` auto-subscribes
3. Graph widgets with live data

---

## Benefits Over v1

| Aspect | v1 (Generic) | v2 (Opinionated) |
|--------|--------------|------------------|
| Code for simple dialog | ~30 lines | ~10 lines |
| Multi-step flow | Manual state tracking | Built-in state machine |
| Complex widgets | Build from scratch | Pre-built, just reference |
| Event handling | Override method | Decorators |
| Native feel | Manual styling | Automatic |
| Learning curve | Medium | Low |
