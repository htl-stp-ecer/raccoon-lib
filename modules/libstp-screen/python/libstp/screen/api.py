import asyncio
import json

import lcm
from typing import Any, Dict
from libstp.screen.exlcm.screen_render_t import screen_render_t
from libstp.screen.exlcm.screen_render_answer_t import screen_render_answer_t
from libstp.hal import AnalogSensor

from libstp.class_name_logger import ClassNameLogger


class RenderScreen(ClassNameLogger):
    def __init__(self, port: int):
        super().__init__()
        self.screen_name = ""
        self.port = port
        self.LCM = lcm.LCM()
        self.cancel_event = asyncio.Event()
        self.sensor = AnalogSensor(port)
        self.LCM.subscribe("libstp/screen_render/cancel", self._handle_cancel_request)
        asyncio.create_task(self._lcm_pump_async())

    def _handle_cancel_request(self, channel, data):
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

    async def wait_for_lcm_message(self, timeout: float = 10.0) -> Any:
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

    async def wait_for_finish(self, timeout: float = 10.0) -> Any:
        return await self.wait_for_lcm_message(timeout=timeout)

    async def wait_for_button(self):
        for _ in range(10):
            if self.cancel_event.is_set():
                self.debug("Cancelled wait_for_button")
                raise asyncio.CancelledError()
            await asyncio.sleep(1)

    def getAnalogValue(self):
        value = self.sensor.read()
        while value == 0:
            value = self.sensor.read()
        return value

    async def calibrate_black_white(self, trie=0) -> None:
        self.cancel_event.clear()
        self.change_screen("calibrate_sensors")

        if self.cancel_event.is_set():
            self.debug("Calibration cancelled before start")
            return

        self.debug("Calibrating black request")
        self.send_state({"port": self.port, "type": "blackWhite", "state": "calibrate_black"})

        await self.wait_for_button()
        black_value = self.getAnalogValue()
        self.debug("Black Value: " + str(black_value))

        self.debug("Calibrating white request")
        self.send_state({"port": self.port, "type": "blackWhite", "state": "calibrate_white"})

        await self.wait_for_button()
        white_value = self.getAnalogValue()
        self.debug("White Value: " + str(white_value))

        self.debug("Time to confirm")
        self.send_state({
            "port": self.port,
            "type": "blackWhite",
            "state": "confirm",
            "white_value": white_value,
            "black_value": black_value
        })

        try:
            msg = await self.wait_for_finish(timeout=120)
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

        print(f"Set values to: black {black_value} white {white_value}")

    async def _lcm_pump_async(self):
        while True:
            self.LCM.handle_timeout(0)
            await asyncio.sleep(0.01)
