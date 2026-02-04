# BotUI Dynamic Screen Protocol Design

A type-safe, annotation-based system for defining UI screens in Python that render in Flutter.

## Design Principles

1. **Python does the logic** - Flutter just renders what it's told
2. **Type-safe** - Dataclasses with full IDE support and validation
3. **Annotation-based** - Follows existing `@dsl` pattern
4. **Simple JSON protocol** - Maps directly to existing Flutter widgets
5. **Step integration** - Screens are Steps that can be composed

---

## Python Side

### Widget Definitions (`libstp/ui/widgets.py`)

```python
from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Union, Literal
from enum import Enum
import json


class WidgetType(str, Enum):
    TEXT = "text"
    BUTTON = "button"
    SLIDER = "slider"
    CHECKBOX = "checkbox"
    DROPDOWN = "dropdown"
    INPUT = "input"
    COUNTER = "counter"
    SPACER = "spacer"
    DIVIDER = "divider"


class ButtonStyle(str, Enum):
    PRIMARY = "primary"
    SECONDARY = "secondary"
    DANGER = "danger"
    SUCCESS = "success"


class TextSize(str, Enum):
    SMALL = "small"
    MEDIUM = "medium"
    LARGE = "large"
    TITLE = "title"


@dataclass
class Widget:
    """Base widget class - all widgets inherit from this."""

    def to_dict(self) -> dict:
        """Convert to JSON-serializable dict."""
        d = asdict(self)
        d["widget_type"] = self.__class__.__name__.lower()
        return d


# ============ Display Widgets ============

@dataclass
class Text(Widget):
    """Static text display."""
    text: str
    size: TextSize = TextSize.MEDIUM
    bold: bool = False
    color: Optional[str] = None  # hex color e.g. "#FF5733"
    align: Literal["left", "center", "right"] = "left"


@dataclass
class Spacer(Widget):
    """Vertical spacing."""
    height: int = 16


@dataclass
class Divider(Widget):
    """Horizontal line divider."""
    thickness: int = 1
    color: Optional[str] = None


# ============ Input Widgets ============

@dataclass
class Button(Widget):
    """Clickable button that submits form or triggers action."""
    id: str
    label: str
    style: ButtonStyle = ButtonStyle.PRIMARY
    icon: Optional[str] = None  # Material icon name
    disabled: bool = False
    # If submit=True, clicking sends all form values
    # If submit=False, only sends this button's id as action
    submit: bool = True


@dataclass
class Slider(Widget):
    """Numeric slider input."""
    id: str
    min: float
    max: float
    value: float = 0
    step: float = 1
    label: Optional[str] = None
    show_value: bool = True


@dataclass
class Counter(Widget):
    """Large +/- counter (maps to LargeButtonCounter)."""
    id: str
    min: float
    max: float
    value: float = 0
    divisions: int = 100
    label: Optional[str] = None


@dataclass
class Checkbox(Widget):
    """Boolean toggle (maps to LargeCheckbox)."""
    id: str
    label: str
    value: bool = False


@dataclass
class Dropdown(Widget):
    """Selection from list (maps to LargeDropdown)."""
    id: str
    options: List[str]
    value: Optional[str] = None
    label: Optional[str] = None
    hint: Optional[str] = None


@dataclass
class Input(Widget):
    """Text input field."""
    id: str
    label: str
    value: str = ""
    placeholder: str = ""
    keyboard_type: Literal["text", "number", "email"] = "text"


# Type alias for any widget
AnyWidget = Union[Text, Button, Slider, Counter, Checkbox, Dropdown, Input, Spacer, Divider]
```

### Layout Containers (`libstp/ui/layout.py`)

```python
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Union, Literal, Optional
from .widgets import AnyWidget


@dataclass
class Row:
    """Horizontal layout - children side by side."""
    children: List[Union[AnyWidget, "Column"]]
    spacing: int = 8
    align: Literal["start", "center", "end", "space_between"] = "center"

    def to_dict(self) -> dict:
        return {
            "layout_type": "row",
            "spacing": self.spacing,
            "align": self.align,
            "children": [
                c.to_dict() if hasattr(c, 'to_dict') else c
                for c in self.children
            ]
        }


@dataclass
class Column:
    """Vertical layout - children stacked."""
    children: List[Union[AnyWidget, "Row"]]
    spacing: int = 12
    align: Literal["start", "center", "end", "stretch"] = "stretch"

    def to_dict(self) -> dict:
        return {
            "layout_type": "column",
            "spacing": self.spacing,
            "align": self.align,
            "children": [
                c.to_dict() if hasattr(c, 'to_dict') else c
                for c in self.children
            ]
        }


@dataclass
class Card:
    """Card container with optional title."""
    children: List[Union[AnyWidget, Row, Column]]
    title: Optional[str] = None
    padding: int = 16

    def to_dict(self) -> dict:
        return {
            "layout_type": "card",
            "title": self.title,
            "padding": self.padding,
            "children": [
                c.to_dict() if hasattr(c, 'to_dict') else c
                for c in self.children
            ]
        }


LayoutItem = Union[AnyWidget, Row, Column, Card]
```

### Screen Definition (`libstp/ui/screen.py`)

```python
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional
import json
from .layout import LayoutItem


@dataclass
class Screen:
    """Complete screen definition."""
    title: str
    body: List[LayoutItem]
    show_back_button: bool = True
    # Optional footer buttons (always at bottom)
    footer: Optional[List[LayoutItem]] = None

    def to_json(self) -> str:
        """Serialize to JSON for LCM transmission."""
        return json.dumps({
            "type": "dynamic_screen",
            "title": self.title,
            "show_back_button": self.show_back_button,
            "body": [item.to_dict() for item in self.body],
            "footer": [item.to_dict() for item in self.footer] if self.footer else None,
        })

    def to_dict(self) -> dict:
        return json.loads(self.to_json())
```

### UiScreen Step Base Class (`libstp/ui/step.py`)

```python
from __future__ import annotations
from abc import abstractmethod
from dataclasses import dataclass
from typing import Dict, Any, Optional, TYPE_CHECKING
import asyncio
import json
import lcm

from libstp.step.base import Step
from libstp.step.annotation import dsl
from libstp.class_name_logger import ClassNameLogger
from .screen import Screen
from libstp.screen.exlcm.screen_render_t import screen_render_t
from libstp.screen.exlcm.screen_render_answer_t import screen_render_answer_t

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dataclass
class UiResponse:
    """Response from the UI."""
    action: str  # Button id that was clicked
    values: Dict[str, Any]  # All form field values {id: value}


class UiScreen(Step, ClassNameLogger):
    """
    Base class for steps that display a UI screen and wait for user interaction.

    Subclass this and implement:
    - build_screen(): Define the screen layout
    - on_response(): Handle user input (optional)

    Example:
        @dsl(tags=["ui", "config"])
        class ConfigureSpeed(UiScreen):
            def __init__(self, default_speed: int = 50):
                super().__init__()
                self.default_speed = default_speed
                self.selected_speed = default_speed

            def build_screen(self) -> Screen:
                return Screen(
                    title="Configure Speed",
                    body=[
                        Text("Select robot speed:"),
                        Slider(id="speed", min=0, max=100, value=self.default_speed),
                    ],
                    footer=[
                        Row([
                            Button(id="cancel", label="Cancel", style=ButtonStyle.SECONDARY),
                            Button(id="confirm", label="Confirm", style=ButtonStyle.PRIMARY),
                        ])
                    ]
                )

            async def on_response(self, response: UiResponse) -> None:
                if response.action == "confirm":
                    self.selected_speed = response.values.get("speed", self.default_speed)
    """

    _screen_name = "dynamic_ui"

    def __init__(self, timeout: float = 120.0):
        super().__init__()
        self.timeout = timeout
        self._lcm = lcm.LCM()
        self._response: Optional[UiResponse] = None

    @abstractmethod
    def build_screen(self) -> Screen:
        """Override to define the screen layout."""
        raise NotImplementedError

    async def on_response(self, response: UiResponse) -> None:
        """
        Override to handle user response.
        Called after user clicks a submit button.
        """
        pass

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Send screen to Flutter and wait for response."""
        screen = self.build_screen()

        # Send screen render request
        msg = screen_render_t()
        msg.screen_name = self._screen_name
        msg.entries = screen.to_json()
        self._lcm.publish("libstp/screen_render", msg.encode())
        self.debug(f"Sent screen: {screen.title}")

        # Wait for response
        response = await self._wait_for_response()
        if response:
            self._response = response
            await self.on_response(response)

    async def _wait_for_response(self) -> Optional[UiResponse]:
        """Wait for LCM response from Flutter UI."""
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def handler(channel, data):
            msg = screen_render_answer_t.decode(data)
            if msg.screen_name == self._screen_name and not future.done():
                try:
                    values = json.loads(msg.reason) if msg.reason else {}
                except json.JSONDecodeError:
                    values = {}
                response = UiResponse(action=msg.value, values=values)
                loop.call_soon_threadsafe(future.set_result, response)

        sub = self._lcm.subscribe("libstp/screen_render/answer", handler)

        # LCM pump task
        async def pump():
            while not future.done():
                self._lcm.handle_timeout(0)
                await asyncio.sleep(0.01)

        pump_task = asyncio.create_task(pump())

        try:
            response = await asyncio.wait_for(future, timeout=self.timeout)
            return response
        except asyncio.TimeoutError:
            self.warn("UI response timeout")
            return None
        finally:
            pump_task.cancel()
            self._lcm.unsubscribe(sub)

    @property
    def response(self) -> Optional[UiResponse]:
        """Get the last UI response after step completes."""
        return self._response
```

### Convenience Factory Functions (`libstp/ui/__init__.py`)

```python
"""
BotUI - Dynamic screen building for robotics.

Example usage:
    from libstp.ui import *

    @dsl(tags=["ui"])
    class AskForConfirmation(UiScreen):
        def __init__(self, message: str):
            super().__init__()
            self.message = message
            self.confirmed = False

        def build_screen(self) -> Screen:
            return Screen(
                title="Confirm",
                body=[
                    Text(self.message, size=TextSize.LARGE),
                ],
                footer=[
                    Row([
                        Button(id="no", label="No", style=ButtonStyle.DANGER),
                        Button(id="yes", label="Yes", style=ButtonStyle.SUCCESS),
                    ])
                ]
            )

        async def on_response(self, response: UiResponse) -> None:
            self.confirmed = response.action == "yes"

    # Factory function for DSL
    @dsl(tags=["ui", "dialog"])
    def confirm(message: str) -> AskForConfirmation:
        return AskForConfirmation(message)
"""

from .widgets import (
    Widget,
    Text, TextSize,
    Button, ButtonStyle,
    Slider,
    Counter,
    Checkbox,
    Dropdown,
    Input,
    Spacer,
    Divider,
)
from .layout import Row, Column, Card
from .screen import Screen
from .step import UiScreen, UiResponse

__all__ = [
    # Widgets
    "Widget",
    "Text", "TextSize",
    "Button", "ButtonStyle",
    "Slider",
    "Counter",
    "Checkbox",
    "Dropdown",
    "Input",
    "Spacer",
    "Divider",
    # Layout
    "Row",
    "Column",
    "Card",
    # Screen
    "Screen",
    # Step
    "UiScreen",
    "UiResponse",
]
```

---

## JSON Protocol

The Python side serializes to this JSON format:

```json
{
  "type": "dynamic_screen",
  "title": "Configure Robot",
  "show_back_button": true,
  "body": [
    {
      "widget_type": "text",
      "text": "Select your settings:",
      "size": "large",
      "bold": true,
      "color": null,
      "align": "center"
    },
    {
      "layout_type": "card",
      "title": "Movement",
      "padding": 16,
      "children": [
        {
          "widget_type": "slider",
          "id": "speed",
          "min": 0,
          "max": 100,
          "value": 50,
          "step": 5,
          "label": "Speed",
          "show_value": true
        },
        {
          "widget_type": "checkbox",
          "id": "turbo",
          "label": "Enable Turbo Mode",
          "value": false
        }
      ]
    },
    {
      "layout_type": "row",
      "spacing": 8,
      "align": "center",
      "children": [
        {
          "widget_type": "dropdown",
          "id": "mode",
          "options": ["Careful", "Normal", "Aggressive"],
          "value": "Normal",
          "label": "Driving Mode"
        }
      ]
    }
  ],
  "footer": [
    {
      "layout_type": "row",
      "spacing": 16,
      "align": "space_between",
      "children": [
        {
          "widget_type": "button",
          "id": "cancel",
          "label": "Cancel",
          "style": "secondary",
          "submit": false
        },
        {
          "widget_type": "button",
          "id": "confirm",
          "label": "Apply Settings",
          "style": "primary",
          "submit": true
        }
      ]
    }
  ]
}
```

### Response Format

Flutter sends back via `screen_render_answer_t`:

```
screen_name: "dynamic_ui"
value: "confirm"  // The button id that was clicked
reason: "{\"speed\": 75, \"turbo\": true, \"mode\": \"Aggressive\"}"  // JSON of all form values
```

---

## Flutter Side

### Widget Decoder (`lib/features/dynamic_ui/decoder/widget_decoder.dart`)

```dart
import 'package:flutter/material.dart';
import 'package:stpvelox/core/widgets/large_checkbox.dart';
import 'package:stpvelox/core/widgets/large_dropdown.dart';
import 'package:stpvelox/core/widgets/large_button.dart';

typedef OnValueChanged = void Function(String id, dynamic value);
typedef OnButtonPressed = void Function(String id, bool submit);

class WidgetDecoder {
  final OnValueChanged onValueChanged;
  final OnButtonPressed onButtonPressed;

  WidgetDecoder({
    required this.onValueChanged,
    required this.onButtonPressed,
  });

  Widget decode(Map<String, dynamic> json) {
    // Check if it's a layout or widget
    if (json.containsKey('layout_type')) {
      return _decodeLayout(json);
    } else if (json.containsKey('widget_type')) {
      return _decodeWidget(json);
    }
    return const SizedBox.shrink();
  }

  Widget _decodeLayout(Map<String, dynamic> json) {
    final type = json['layout_type'] as String;
    final children = (json['children'] as List)
        .map((c) => decode(c as Map<String, dynamic>))
        .toList();

    switch (type) {
      case 'row':
        return Row(
          mainAxisAlignment: _parseMainAxis(json['align'] as String? ?? 'center'),
          children: _addSpacing(children, json['spacing'] as int? ?? 8, Axis.horizontal),
        );

      case 'column':
        return Column(
          crossAxisAlignment: _parseCrossAxis(json['align'] as String? ?? 'stretch'),
          children: _addSpacing(children, json['spacing'] as int? ?? 12, Axis.vertical),
        );

      case 'card':
        final title = json['title'] as String?;
        final padding = json['padding'] as int? ?? 16;
        return Card(
          child: Padding(
            padding: EdgeInsets.all(padding.toDouble()),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                if (title != null) ...[
                  Text(title, style: const TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                  )),
                  const SizedBox(height: 12),
                ],
                ...children,
              ],
            ),
          ),
        );

      default:
        return Column(children: children);
    }
  }

  Widget _decodeWidget(Map<String, dynamic> json) {
    final type = json['widget_type'] as String;

    switch (type) {
      case 'text':
        return _buildText(json);
      case 'button':
        return _buildButton(json);
      case 'slider':
        return _buildSlider(json);
      case 'counter':
        return _buildCounter(json);
      case 'checkbox':
        return _buildCheckbox(json);
      case 'dropdown':
        return _buildDropdown(json);
      case 'input':
        return _buildInput(json);
      case 'spacer':
        return SizedBox(height: (json['height'] as int? ?? 16).toDouble());
      case 'divider':
        return Divider(
          thickness: (json['thickness'] as int? ?? 1).toDouble(),
          color: json['color'] != null ? _parseColor(json['color']) : null,
        );
      default:
        return const SizedBox.shrink();
    }
  }

  Widget _buildText(Map<String, dynamic> json) {
    final size = json['size'] as String? ?? 'medium';
    final fontSize = switch (size) {
      'small' => 14.0,
      'medium' => 18.0,
      'large' => 24.0,
      'title' => 32.0,
      _ => 18.0,
    };

    return Text(
      json['text'] as String,
      textAlign: _parseTextAlign(json['align'] as String? ?? 'left'),
      style: TextStyle(
        fontSize: fontSize,
        fontWeight: (json['bold'] as bool? ?? false) ? FontWeight.bold : FontWeight.normal,
        color: json['color'] != null ? _parseColor(json['color']) : null,
      ),
    );
  }

  Widget _buildButton(Map<String, dynamic> json) {
    final id = json['id'] as String;
    final label = json['label'] as String;
    final style = json['style'] as String? ?? 'primary';
    final disabled = json['disabled'] as bool? ?? false;
    final submit = json['submit'] as bool? ?? true;
    final icon = json['icon'] as String?;

    final color = switch (style) {
      'primary' => Colors.blue,
      'secondary' => Colors.grey,
      'danger' => Colors.red,
      'success' => Colors.green,
      _ => Colors.blue,
    };

    return ElevatedButton.icon(
      onPressed: disabled ? null : () => onButtonPressed(id, submit),
      style: ElevatedButton.styleFrom(
        backgroundColor: color,
        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
      ),
      icon: icon != null ? Icon(_parseIcon(icon)) : const SizedBox.shrink(),
      label: Text(label, style: const TextStyle(fontSize: 18)),
    );
  }

  Widget _buildSlider(Map<String, dynamic> json) {
    final id = json['id'] as String;
    final label = json['label'] as String?;
    final min = (json['min'] as num).toDouble();
    final max = (json['max'] as num).toDouble();
    final value = (json['value'] as num? ?? min).toDouble();
    final step = (json['step'] as num? ?? 1).toDouble();
    final showValue = json['show_value'] as bool? ?? true;

    return _SliderWidget(
      id: id,
      label: label,
      min: min,
      max: max,
      initialValue: value,
      divisions: ((max - min) / step).round(),
      showValue: showValue,
      onChanged: (v) => onValueChanged(id, v),
    );
  }

  Widget _buildCounter(Map<String, dynamic> json) {
    final id = json['id'] as String;
    final label = json['label'] as String?;

    return Column(
      children: [
        if (label != null) Text(label, style: const TextStyle(fontSize: 16)),
        LargeButtonCounter(
          min: (json['min'] as num).toDouble(),
          max: (json['max'] as num).toDouble(),
          initial: (json['value'] as num? ?? 0).toDouble(),
          divisions: json['divisions'] as int? ?? 100,
          onChanged: (v) => onValueChanged(id, v),
        ),
      ],
    );
  }

  Widget _buildCheckbox(Map<String, dynamic> json) {
    final id = json['id'] as String;
    return LargeCheckbox(
      label: json['label'] as String?,
      initialValue: json['value'] as bool? ?? false,
      onChanged: (v) => onValueChanged(id, v),
    );
  }

  Widget _buildDropdown(Map<String, dynamic> json) {
    final id = json['id'] as String;
    final options = (json['options'] as List).cast<String>();

    return LargeDropdown(
      options: options,
      initialSelected: json['value'] as String?,
      label: json['label'] as String?,
      hint: json['hint'] as String?,
      onChanged: (v) => onValueChanged(id, v),
    );
  }

  Widget _buildInput(Map<String, dynamic> json) {
    final id = json['id'] as String;
    final keyboardType = switch (json['keyboard_type'] as String? ?? 'text') {
      'number' => TextInputType.number,
      'email' => TextInputType.emailAddress,
      _ => TextInputType.text,
    };

    return TextField(
      decoration: InputDecoration(
        labelText: json['label'] as String?,
        hintText: json['placeholder'] as String?,
        border: const OutlineInputBorder(),
      ),
      keyboardType: keyboardType,
      controller: TextEditingController(text: json['value'] as String? ?? ''),
      onChanged: (v) => onValueChanged(id, v),
      style: const TextStyle(fontSize: 18),
    );
  }

  // Helper methods
  MainAxisAlignment _parseMainAxis(String align) => switch (align) {
    'start' => MainAxisAlignment.start,
    'center' => MainAxisAlignment.center,
    'end' => MainAxisAlignment.end,
    'space_between' => MainAxisAlignment.spaceBetween,
    _ => MainAxisAlignment.center,
  };

  CrossAxisAlignment _parseCrossAxis(String align) => switch (align) {
    'start' => CrossAxisAlignment.start,
    'center' => CrossAxisAlignment.center,
    'end' => CrossAxisAlignment.end,
    'stretch' => CrossAxisAlignment.stretch,
    _ => CrossAxisAlignment.stretch,
  };

  TextAlign _parseTextAlign(String align) => switch (align) {
    'left' => TextAlign.left,
    'center' => TextAlign.center,
    'right' => TextAlign.right,
    _ => TextAlign.left,
  };

  Color _parseColor(String hex) {
    final buffer = StringBuffer();
    if (hex.length == 6 || hex.length == 7) buffer.write('ff');
    buffer.write(hex.replaceFirst('#', ''));
    return Color(int.parse(buffer.toString(), radix: 16));
  }

  IconData _parseIcon(String name) {
    // Map common icon names to Material icons
    return switch (name) {
      'check' => Icons.check,
      'close' => Icons.close,
      'arrow_back' => Icons.arrow_back,
      'arrow_forward' => Icons.arrow_forward,
      'settings' => Icons.settings,
      'play' => Icons.play_arrow,
      'stop' => Icons.stop,
      'refresh' => Icons.refresh,
      _ => Icons.help_outline,
    };
  }

  List<Widget> _addSpacing(List<Widget> children, int spacing, Axis axis) {
    if (children.isEmpty) return children;
    final spacer = axis == Axis.horizontal
        ? SizedBox(width: spacing.toDouble())
        : SizedBox(height: spacing.toDouble());

    return children.expand((w) => [w, spacer]).toList()..removeLast();
  }
}

// Stateful slider widget
class _SliderWidget extends StatefulWidget {
  final String id;
  final String? label;
  final double min;
  final double max;
  final double initialValue;
  final int divisions;
  final bool showValue;
  final ValueChanged<double> onChanged;

  const _SliderWidget({
    required this.id,
    this.label,
    required this.min,
    required this.max,
    required this.initialValue,
    required this.divisions,
    required this.showValue,
    required this.onChanged,
  });

  @override
  State<_SliderWidget> createState() => _SliderWidgetState();
}

class _SliderWidgetState extends State<_SliderWidget> {
  late double _value;

  @override
  void initState() {
    super.initState();
    _value = widget.initialValue;
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        if (widget.label != null)
          Padding(
            padding: const EdgeInsets.only(bottom: 8),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text(widget.label!, style: const TextStyle(fontSize: 16)),
                if (widget.showValue)
                  Text(_value.toStringAsFixed(1),
                       style: const TextStyle(fontSize: 16, fontWeight: FontWeight.bold)),
              ],
            ),
          ),
        Slider(
          value: _value,
          min: widget.min,
          max: widget.max,
          divisions: widget.divisions,
          onChanged: (v) {
            setState(() => _value = v);
            widget.onChanged(v);
          },
        ),
      ],
    );
  }
}
```

### Dynamic Screen Widget (`lib/features/dynamic_ui/presentation/dynamic_screen.dart`)

```dart
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:stpvelox/core/lcm/domain/providers.dart';
import 'package:stpvelox/core/widgets/top_bar.dart';
import 'package:stpvelox/lcm/types/screen_render_answer_t.g.dart';
import '../decoder/widget_decoder.dart';

class DynamicScreen extends ConsumerStatefulWidget {
  final Map<String, dynamic> screenData;

  const DynamicScreen({super.key, required this.screenData});

  @override
  ConsumerState<DynamicScreen> createState() => _DynamicScreenState();
}

class _DynamicScreenState extends ConsumerState<DynamicScreen> {
  final Map<String, dynamic> _formValues = {};
  late WidgetDecoder _decoder;

  @override
  void initState() {
    super.initState();
    _decoder = WidgetDecoder(
      onValueChanged: _handleValueChanged,
      onButtonPressed: _handleButtonPressed,
    );
    _initializeFormValues();
  }

  void _initializeFormValues() {
    // Extract initial values from widget definitions
    void extractValues(dynamic item) {
      if (item is Map<String, dynamic>) {
        if (item.containsKey('id') && item.containsKey('value')) {
          _formValues[item['id'] as String] = item['value'];
        }
        if (item.containsKey('children')) {
          for (final child in item['children'] as List) {
            extractValues(child);
          }
        }
      }
    }

    for (final item in widget.screenData['body'] as List? ?? []) {
      extractValues(item);
    }
    for (final item in widget.screenData['footer'] as List? ?? []) {
      extractValues(item);
    }
  }

  void _handleValueChanged(String id, dynamic value) {
    setState(() {
      _formValues[id] = value;
    });
  }

  void _handleButtonPressed(String id, bool submit) {
    final lcm = ref.read(lcmServiceProvider);

    final response = ScreenRenderAnswerT(
      screen_name: 'dynamic_ui',
      value: id,
      reason: submit ? jsonEncode(_formValues) : '',
    );

    lcm.publish('libstp/screen_render/answer', response);

    // Optionally pop the screen
    if (context.mounted) {
      Navigator.of(context).pop();
    }
  }

  @override
  Widget build(BuildContext context) {
    final title = widget.screenData['title'] as String? ?? 'Screen';
    final showBack = widget.screenData['show_back_button'] as bool? ?? true;
    final body = widget.screenData['body'] as List? ?? [];
    final footer = widget.screenData['footer'] as List?;

    return Scaffold(
      appBar: createTopBar(
        context,
        title,
        showBack: showBack,
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            Expanded(
              child: SingleChildScrollView(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: body
                      .map((item) => _decoder.decode(item as Map<String, dynamic>))
                      .toList(),
                ),
              ),
            ),
            if (footer != null) ...[
              const Divider(),
              Padding(
                padding: const EdgeInsets.only(top: 8),
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                  children: footer
                      .map((item) => _decoder.decode(item as Map<String, dynamic>))
                      .toList(),
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }
}
```

### Integration with Screen Renderer Provider

Add to `screen_renderer_provider.dart`:

```dart
// In _startSubscription(), add a case for dynamic_screen:
if (parsed['type'] == 'dynamic_screen') {
  await handleDynamicScreen(parsed);
}

Future<void> handleDynamicScreen(Map<String, dynamic> parsed) async {
  state = DynamicScreen(screenData: parsed);
}
```

---

## Complete Usage Example

### Python: Custom Configuration Step

```python
from libstp.step import dsl
from libstp.ui import *


@dsl(tags=["ui", "config"])
class ConfigureRobot(UiScreen):
    """Interactive configuration screen for robot settings."""

    def __init__(self,
                 default_speed: int = 50,
                 default_mode: str = "Normal"):
        super().__init__()
        self.default_speed = default_speed
        self.default_mode = default_mode
        # These will be set after user interaction
        self.speed: int = default_speed
        self.mode: str = default_mode
        self.turbo: bool = False

    def build_screen(self) -> Screen:
        return Screen(
            title="Robot Configuration",
            body=[
                Text("Configure your robot before starting",
                     size=TextSize.LARGE, align="center"),
                Spacer(height=24),

                Card(title="Movement Settings", children=[
                    Slider(
                        id="speed",
                        label="Speed",
                        min=0,
                        max=100,
                        value=self.default_speed,
                        step=5,
                    ),
                    Spacer(),
                    Checkbox(
                        id="turbo",
                        label="Enable Turbo Mode (use with caution!)",
                        value=False,
                    ),
                ]),

                Spacer(),

                Card(title="Behavior", children=[
                    Dropdown(
                        id="mode",
                        label="Driving Mode",
                        options=["Careful", "Normal", "Aggressive"],
                        value=self.default_mode,
                    ),
                ]),
            ],
            footer=[
                Row(align="space_between", children=[
                    Button(id="cancel", label="Cancel",
                           style=ButtonStyle.SECONDARY, submit=False),
                    Button(id="start", label="Start Robot",
                           style=ButtonStyle.SUCCESS, icon="play"),
                ])
            ]
        )

    async def on_response(self, response: UiResponse) -> None:
        if response.action == "start":
            self.speed = int(response.values.get("speed", self.default_speed))
            self.mode = response.values.get("mode", self.default_mode)
            self.turbo = response.values.get("turbo", False)
            self.info(f"Config: speed={self.speed}, mode={self.mode}, turbo={self.turbo}")


# Factory function
@dsl(name="Configure Robot", tags=["ui", "config"])
def configure_robot(speed: int = 50, mode: str = "Normal") -> ConfigureRobot:
    """Show robot configuration screen."""
    return ConfigureRobot(default_speed=speed, default_mode=mode)
```

### Python: Using in a Mission

```python
from libstp import *
from libstp.ui import *


class MyMission(Mission):
    def sequence(self) -> Step:
        # Create config step to access values later
        config = configure_robot(speed=75)

        return Sequential([
            # Show config screen first
            config,

            # Use configured values
            # Note: In real usage, you'd access config.speed etc.
            drive_forward(100),
            turn_left(90),
        ])
```

### Python: Simple Confirmation Dialog

```python
@dsl(tags=["ui", "dialog"])
class ConfirmDialog(UiScreen):
    def __init__(self, title: str, message: str):
        super().__init__()
        self.title = title
        self.message = message
        self.confirmed = False

    def build_screen(self) -> Screen:
        return Screen(
            title=self.title,
            show_back_button=False,
            body=[
                Spacer(height=40),
                Text(self.message, size=TextSize.LARGE, align="center"),
                Spacer(height=40),
            ],
            footer=[
                Row(spacing=24, children=[
                    Button(id="no", label="No", style=ButtonStyle.DANGER),
                    Button(id="yes", label="Yes", style=ButtonStyle.SUCCESS),
                ])
            ]
        )

    async def on_response(self, response: UiResponse) -> None:
        self.confirmed = response.action == "yes"


@dsl(name="Confirm", tags=["ui", "dialog"])
def confirm(title: str = "Confirm", message: str = "Are you sure?") -> ConfirmDialog:
    return ConfirmDialog(title, message)
```

---

## Widget Reference

| Widget | Flutter Component | Properties |
|--------|------------------|------------|
| `Text` | `Text` | text, size, bold, color, align |
| `Button` | `ElevatedButton` | id, label, style, icon, disabled, submit |
| `Slider` | `Slider` | id, min, max, value, step, label, show_value |
| `Counter` | `LargeButtonCounter` | id, min, max, value, divisions, label |
| `Checkbox` | `LargeCheckbox` | id, label, value |
| `Dropdown` | `LargeDropdown` | id, options, value, label, hint |
| `Input` | `TextField` | id, label, value, placeholder, keyboard_type |
| `Spacer` | `SizedBox` | height |
| `Divider` | `Divider` | thickness, color |

| Layout | Description | Properties |
|--------|-------------|------------|
| `Row` | Horizontal | children, spacing, align |
| `Column` | Vertical | children, spacing, align |
| `Card` | Container | children, title, padding |

| Button Style | Color |
|--------------|-------|
| `primary` | Blue |
| `secondary` | Grey |
| `danger` | Red |
| `success` | Green |

| Text Size | Font Size |
|-----------|-----------|
| `small` | 14px |
| `medium` | 18px |
| `large` | 24px |
| `title` | 32px |
