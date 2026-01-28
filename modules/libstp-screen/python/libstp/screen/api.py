import asyncio
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import signal
import threading
import lcm
from libstp.screen.exlcm.screen_render_t import screen_render_t
from libstp.screen.exlcm.screen_render_answer_t import screen_render_answer_t
from libstp.hal import AnalogSensor
from libstp.sensor_ir import IRSensor
from libstp.sensor_ir import IRSensorCalibration
from libstp.class_name_logger import ClassNameLogger
from libstp import button as _button
from libstp import calibration_store as CalibrationStore
from libstp.calibration_store import CalibrationType

import os


@dataclass
class WFLCalibrationResult:
    """Result of wait-for-light sensor calibration."""
    light_off: float  # Sensor value when covered (dark)
    light_on: float  # Sensor value when exposed to light
    threshold: float  # Computed threshold for detection


@dataclass
class IRSensorCalibrationResult:
    whiteThresh: float
    blackThresh: float


@dataclass
class DistanceCalibrationResult:
    """Result of distance calibration."""
    requested_distance_cm: float  # What we asked the robot to drive
    measured_distance_cm: float  # What the human measured
    scale_factor: float  # measured / requested


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
        self._button_cancel_token = threading.Event()
        self._calibration_task = None
        self.LCM.subscribe("libstp/screen_render/cancel", self.__handle_cancel_request)
        asyncio.create_task(self.__lcm_pump_async())
        self._original_sigint_handler = signal.getsignal(signal.SIGINT)

    def __handle_sigint(self, signum, frame):
        self.cancel_event.set()
        self._button_cancel_token.set()
        if self._calibration_task:
            self._calibration_task.cancel()
        if callable(self._original_sigint_handler):
            self._original_sigint_handler(signum, frame)

    def __handle_cancel_request(self, channel, data):
        self.warn("Something went wrong! Try check the data you send!")
        self.cancel_event.set()
        self._button_cancel_token.set()
        if self._calibration_task:
            self._calibration_task.cancel()

    def change_screen(self, new_screen: str):
        self.screen_name = new_screen

    def __send_screen_render_request_to_lcm(self, data: Dict[str, Any]):
        msg = screen_render_t()
        msg.screen_name = self.screen_name
        msg.entries = json.dumps(data)
        self.LCM.publish("libstp/screen_render", msg.encode())

    def send_state(self, data: Dict[str, Any]):
        self.__send_screen_render_request_to_lcm(data)

    async def __wait_for_button(self, button_port=10):
        self.info("Press the button when ready.")
        try:
            while True:
                pressed = _button.is_pressed()
                if pressed:
                    return True
                await asyncio.sleep(0.01)
        except asyncio.CancelledError:
            return False

    async def __wait_for_lcm_response(self, timeout: float = 10.0) -> LCMResponse:
        """Wait for LCM message and return both value and reason."""
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def handler(channel, data):
            msg = screen_render_answer_t.decode(data)
            self.info(msg.screen_name)
            if msg.screen_name == self.screen_name and not future.done():
                self.debug(msg.reason)
                loop.call_soon_threadsafe(future.set_result, msg)

        sub = self.LCM.subscribe("libstp/screen_render/answer", handler)

        try:
            msg = await asyncio.wait_for(future, timeout=timeout)
            self.info("Message received")
            return LCMResponse(value=msg.value, reason=msg.reason)
        except asyncio.TimeoutError:
            return LCMResponse(value="retry", reason="timeout")
        finally:
            self.LCM.unsubscribe(sub)


    async def __calibrate_sensors_request(self, usePre=False, button_port=10, trie=0, MAX_ATTEMPTS=5):
        if not usePre:
            self.info(f"Calibration Request: Attempt {trie} / {MAX_ATTEMPTS}")
            if trie == 0:
                self.send_state({"type": "IR", "state": "overview"})
                _button.set_digital(button_port)
            if self.cancel_event.is_set():
                return "canceled"

            if not await self.__wait_for_button(button_port):
                return "canceled"

        loop = asyncio.get_running_loop()

        try:
            result = await loop.run_in_executor(
                None,
                IRSensorCalibration.calibrateSensors,
                self.sensors,
                5.0,
                usePre
            )
        except asyncio.CancelledError:
            return "canceled"

        if not result:
            trie += 1
            if trie >= MAX_ATTEMPTS:
                self.send_state({"type": "IR", "state": "tooManyAttempts"})
                return "tooManyAttempts"

            self.send_state({"type": "IR", "state": "retrying"})
            return await self.__calibrate_sensors_request(
                usePre, button_port, trie, MAX_ATTEMPTS
            )

        return "success"

    def __check_for_stored_readings(self, calibrationType: CalibrationType):
        return CalibrationStore.has_readings(calibrationType)

    async def calibrate_black_white(self, trie=0, MAX_ATTEMPTS=5) -> Optional[IRSensorCalibrationResult]:
        self.cancel_event.clear()
        self._calibration_task = asyncio.current_task()
        self.change_screen("calibrate_sensors")
        self.send_state({"type": "IR", "state": "modeChoice",
                         "hasValues": self.__check_for_stored_readings(CalibrationType.IR_SENSOR)})

        try:
            mode = await self.__wait_for_lcm_response(timeout=120)
        except asyncio.CancelledError:
            self.send_state({"type": "IR", "state": "canceled"})
            return None
        use_pre = mode.value == "useExisting"
        result = await self.__calibrate_sensors_request(
            usePre=use_pre,
            MAX_ATTEMPTS=MAX_ATTEMPTS
        )

        if result == "tooManyAttempts":
            self.info("Calibration failed: too many attempts")
            return None

        if result != "success":
            self.info("Calibration canceled")
            self.send_state({"type": "IR", "state": "canceled"})
            return None

        try:
            msg = await self.__wait_for_lcm_response(timeout=120)
        except asyncio.CancelledError:
            self.send_state({"type": "IR", "state": "canceled"})
            return None
        except asyncio.TimeoutError:
            msg = LCMResponse(value="retry", reason="Timeout")

        if msg.value == "retry":
            trie += 1
            if trie >= 5:
                return None
            return await self.calibrate_black_white(trie)

        return IRSensorCalibrationResult(
            whiteThresh=self.sensors[0].whiteThreshold,
            blackThresh=self.sensors[0].blackThreshold
        )

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
        self._calibration_task = asyncio.current_task()
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
            "port": port, "type": "waitForLight", "state": "confirm", "wfl_off_value": wfl_off_value,
            "wfl_on_value": wfl_on_value})
        try:
            msg = await self.__wait_for_finish(timeout=120)
        except asyncio.CancelledError:
            return None
        except asyncio.TimeoutError:
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
