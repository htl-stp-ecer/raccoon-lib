import asyncio
import json
import signal
import threading
import lcm
from typing import Any, Dict, List
from libstp.screen.exlcm.screen_render_t import screen_render_t
from libstp.screen.exlcm.screen_render_answer_t import screen_render_answer_t
from libstp.hal import AnalogSensor
from libstp.sensor_ir import IRSensor
from libstp.sensor_ir import IRSensorCalibration
from libstp.class_name_logger import ClassNameLogger
from libstp import button as _button


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
        signal.signal(signal.SIGINT, self.__handle_sigint)

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

    async def __wait_for_lcm_message(self, timeout = 10.0):
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        def handler(channel, data):
            msg = screen_render_answer_t.decode(data)
            if msg.screen_name == self.screen_name and not future.done():
                loop.call_soon_threadsafe(future.set_result, msg)
        sub = self.LCM.subscribe("libstp/screen_render/answer", handler)
        try:
            while not self.cancel_event.is_set():
                try:
                    msg = await asyncio.wait_for(future, timeout=timeout)
                    self.info(msg)
                    return msg.value
                except asyncio.TimeoutError:
                    return "retry"
            raise asyncio.CancelledError()
        finally:
            self.LCM.unsubscribe(sub)

    async def __wait_for_finish(self, timeout = 10.0):
        return await self.__wait_for_lcm_message(timeout=timeout)

    async def __calibrateSensorsRequest(self, button_port=10, trie=0, MAX_ATTEMPTS=5):
        self.info("Calibration Request: Attempt " + str(trie) + " / " + str(MAX_ATTEMPTS))
        if trie == 0:
            _button.set_digital(button_port)

        if self.cancel_event.is_set():
            return False
        if not await self.__wait_for_button(button_port):
            return False
        loop = asyncio.get_running_loop()
        try:
            result = await loop.run_in_executor(None, IRSensorCalibration.calibrateSensors, self.sensors, 5.0)
        except asyncio.CancelledError:
            return False
        if not result:
            trie += 1
            if trie >= MAX_ATTEMPTS:
                self.send_state({"type": "IR", "state": "tooManyAttempts"})
                return False
            return await self.__calibrateSensorsRequest(button_port, trie, MAX_ATTEMPTS)
        return True

    async def calibrate_black_white(self, trie=0, MAX_ATTEMPTS=5):
        self.cancel_event.clear()
        self._calibration_task = asyncio.current_task()
        self.change_screen("calibrate_sensors")
        self.send_state({"type": "IR", "state": "overview"})
        result = await self.__calibrateSensorsRequest(MAX_ATTEMPTS=MAX_ATTEMPTS)
        if not result:
            self.info("Calibration canceled")
            self.send_state({"type": "IR", "state": "canceled"})
            return
        try:
            msg = await self.__wait_for_finish(timeout=120)
        except asyncio.CancelledError:
            self.send_state({"type": "IR", "state": "canceled"})
            return
        except asyncio.TimeoutError:
            msg = "retry"
        if msg == "retry":
            trie += 1
            if trie >= 5:
                return
            await self.calibrate_black_white(trie)

    async def calibrate_wfl(self, trie=0):
        self.cancel_event.clear()
        self._calibration_task = asyncio.current_task()
        self.change_screen("calibrate_sensors")
        self.send_state({"port": self.port, "type": "waitForLight", "state": "calibrate_wfl_off"})
        if not await self.__wait_for_button():
            return
        wfl_off_value = self.__get_analog_value()
        self.send_state({"port": self.port, "type": "waitForLight", "state": "calibrate_wfl_on"})
        if not await self.__wait_for_button():
            return
        wfl_on_value = self.__get_analog_value()
        self.send_state({"port": self.port, "type": "waitForLight", "state": "confirm", "wfl_off_value": wfl_off_value, "wfl_on_value": wfl_on_value})
        try:
            msg = await self.__wait_for_finish(timeout=120)
        except asyncio.CancelledError:
            return
        except asyncio.TimeoutError:
            msg = "retry"
        if msg == "retry":
            trie += 1
            if trie >= 5:
                return
            await self.calibrate_wfl(trie)

    async def __lcm_pump_async(self):
        while True:
            self.LCM.handle_timeout(0)
            await asyncio.sleep(0.01)
