import asyncio
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import lcm
from libstp.screen.exlcm.screen_render_t import screen_render_t
from libstp.screen.exlcm.screen_render_answer_t import screen_render_answer_t
from libstp.hal import AnalogSensor
from libstp.sensor_ir import IRSensor
from libstp.sensor_ir import IRSensorCalibration
from libstp.class_name_logger import ClassNameLogger
from libstp import button as _button


@dataclass
class WFLCalibrationResult:
    """Result of wait-for-light sensor calibration."""
    light_off: float    # Sensor value when covered (dark)
    light_on: float     # Sensor value when exposed to light
    threshold: float    # Computed threshold for detection


@dataclass
class DistanceCalibrationResult:
    """Result of distance calibration."""
    requested_distance_cm: float    # What we asked the robot to drive
    measured_distance_cm: float     # What the human measured
    scale_factor: float             # measured / requested


@dataclass
class LCMResponse:
    """Response from an LCM message."""
    value: str
    reason: str

class RenderScreen(ClassNameLogger):
    def __init__(self, sensors: List[IRSensor]):
        super().__init__()
        self.screen_name = ""
        self.sensors = sensors
        self.LCM = lcm.LCM()
        self.cancel_event = asyncio.Event()
        self.LCM.subscribe("libstp/screen_render/cancel", self.__handle_cancel_request)
        asyncio.create_task(self.__lcm_pump_async())

    def __handle_cancel_request(self, channel, data):
        self.warn("Something went wrong! Try check the data you send!")
        self.cancel_event.set()

    def change_screen(self, new_screen: str):
        self.screen_name = new_screen

    def __send_screen_render_request_to_lcm(self, data: Dict[str, Any]):
        msg = screen_render_t()
        msg.screen_name = self.screen_name
        msg.entries = json.dumps(data)
        self.LCM.publish("libstp/screen_render", msg.encode())

    def send_state(self, data: Dict[str, Any]):
        self.__send_screen_render_request_to_lcm(data)

    async def __wait_for_lcm_message(self, timeout: float = 10.0) -> Any:
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def handler(channel, data):
            msg = screen_render_answer_t.decode(data)
            if msg.screen_name == self.screen_name and not future.done():
                self.debug(msg.reason)
                loop.call_soon_threadsafe(future.set_result, msg)

        sub = self.LCM.subscribe("libstp/screen_render/answer", handler)

        try:
            msg = await asyncio.wait_for(future, timeout=timeout)
            return msg.value
        except asyncio.TimeoutError:
            return "retry"
        finally:
            self.LCM.unsubscribe(sub)

    async def __wait_for_finish(self, timeout: float = 10.0) -> Any:
        return await self.__wait_for_lcm_message(timeout=timeout)

    async def __wait_for_lcm_response(self, timeout: float = 10.0) -> LCMResponse:
        """Wait for LCM message and return both value and reason."""
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def handler(channel, data):
            msg = screen_render_answer_t.decode(data)
            if msg.screen_name == self.screen_name and not future.done():
                self.debug(msg.reason)
                loop.call_soon_threadsafe(future.set_result, msg)

        sub = self.LCM.subscribe("libstp/screen_render/answer", handler)

        try:
            msg = await asyncio.wait_for(future, timeout=timeout)
            return LCMResponse(value=msg.value, reason=msg.reason)
        except asyncio.TimeoutError:
            return LCMResponse(value="retry", reason="timeout")
        finally:
            self.LCM.unsubscribe(sub)

    async def __wait_for_button(self):
        _button.wait_for_button_press()
#        for _ in range(10):
#            if self.cancel_event.is_set():
#                self.debug("Cancelled wait_for_button")
#                raise asyncio.CancelledError()
#            await asyncio.sleep(1)

    async def calibrate_black_white(self, trie=0) -> None:
        self.cancel_event.clear()
        self.change_screen("calibrate_sensors")

        if self.cancel_event.is_set():
            self.debug("Black White Sensor calibration cancelled before start")
            return

        self.debug("Overview calibration screen request")
        self.send_state({"type": "IR", "state": "overview"})

        if not IRSensorCalibration().calibrateSensors(self.sensors, 5.0):
            return
        self.debug("Time to confirm")


        try:
            msg = await self.__wait_for_finish(timeout=120)
        except asyncio.CancelledError:
            self.debug("Calibration cancelled during wait_for_finish")
            return
        except asyncio.TimeoutError:
            self.debug("Calibration timeout during wait_for_finish")
            msg = "retry"

        if msg == "retry":
            trie += 1
            if trie >= 5:
                self.warn("Could not finish calibrating sensors")
                return
            await self.calibrate_black_white(trie)

    async def calibrate_wfl(self, sensor: AnalogSensor, trie: int = 0) -> Optional[WFLCalibrationResult]:
        """
        Calibrate a wait-for-light sensor.

        Args:
            sensor: The AnalogSensor instance to calibrate
            trie: Internal retry counter

        Returns:
            WFLCalibrationResult with light values and threshold, or None if cancelled
        """
        self.cancel_event.clear()
        self.change_screen("calibrate_sensors")

        if self.cancel_event.is_set():
            self.debug("Wait for light calibration cancelled before start")
            return None

        # Get port from sensor for frontend display
        port = getattr(sensor, 'port', 0)

        # State 1: Cover sensor (light off)
        self.debug("Calibrating wfl off request")
        self.send_state({"port": port, "type": "waitForLight", "state": "calibrate_wfl_off"})

        await self.__wait_for_button()
        wfl_off_value = sensor.read()
        self.debug(f"Wait for light off Value: {wfl_off_value}")

        # State 2: Expose sensor to light (light on)
        self.debug("Calibrating wfl on request")
        self.send_state({"port": port, "type": "waitForLight", "state": "calibrate_wfl_on"})

        await self.__wait_for_button()
        wfl_on_value = sensor.read()
        self.debug(f"Wait for light on Value: {wfl_on_value}")

        # State 3: Confirm with editable values
        self.send_state({
            "port": port,
            "type": "waitForLight",
            "state": "confirm",
            "wfl_off_value": wfl_off_value,
            "wfl_on_value": wfl_on_value
        })

        try:
            msg = await self.__wait_for_finish(timeout=120)
        except asyncio.CancelledError:
            self.debug("Calibration cancelled during wait_for_finish")
            return None
        except asyncio.TimeoutError:
            self.debug("Calibration timeout during wait_for_finish")
            msg = "retry"

        if msg == "retry":
            trie += 1
            if trie >= 5:
                self.warn("Could not finish wait for light calibration")
                return None
            return await self.calibrate_wfl(sensor, trie)

        # Calculate threshold as midpoint between off and on values
        threshold = (wfl_off_value + wfl_on_value) / 2.0
        self.debug(f"WFL calibration complete: off={wfl_off_value}, on={wfl_on_value}, threshold={threshold}")

        return WFLCalibrationResult(
            light_off=wfl_off_value,
            light_on=wfl_on_value,
            threshold=threshold
        )


    async def __lcm_pump_async(self):
        while True:
            self.LCM.handle_timeout(0)
            await asyncio.sleep(0.01)
