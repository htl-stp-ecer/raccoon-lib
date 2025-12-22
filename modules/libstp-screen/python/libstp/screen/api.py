import asyncio
import json

import lcm
from typing import Any, Dict
from libstp.screen.exlcm.screen_render_t import screen_render_t
from libstp.screen.exlcm.screen_render_answer_t import screen_render_answer_t
from libstp.hal import AnalogSensor
from libstp.sensor_ir import IRSensor
from libstp.sensor_ir import IRSensorCalibration
from libstp.class_name_logger import ClassNameLogger
from libstp.button import Button

class RenderScreen(ClassNameLogger):
    def __init__(self, port: int | list[int]):
        super().__init__()
        self.screen_name = ""
        self.port = port
        self.LCM = lcm.LCM()
        self.cancel_event = asyncio.Event()
        if (isinstance(port, int)):
            self.sensors = [IRSensor(port)]
        else:
            self.sensors = []
            for i in port:
                self.sensors.append(IRSensor(i))
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

    async def __wait_for_button(self):
        await Button.wait_for_button_press()
#        for _ in range(10):
#            if self.cancel_event.is_set():
#                self.debug("Cancelled wait_for_button")
#                raise asyncio.CancelledError()
#            await asyncio.sleep(1)

    def __get_analog_value(self):
        value = self.sensors.read()
        while value == 0:
            value = self.sensors.read()
        return value

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

    async def calibrate_wfl(self, trie=0):
        self.cancel_event.clear()
        self.change_screen("calibrate_sensors")

        if self.cancel_event.is_set():
            self.debug("Wait for light calibration cancelled before start")
            return

        self.debug("Calibrating wfl off request")
        self.send_state({"port": self.port, "type": "waitForLight", "state": "calibrate_wfl_off"})

        await self.__wait_for_button()
        wfl_off_value = self.__get_analog_value()
        self.debug("Wait for light off Value: " + str(wfl_off_value))

        self.debug("Calibrating wfl on request")
        self.send_state({"port": self.port, "type": "waitForLight", "state": "calibrate_wfl_on"})

        await self.__wait_for_button()
        wfl_on_value = self.__get_analog_value()
        self.debug("Wait for light on Value: " + str(wfl_on_value))

        self.send_state({"port": self.port, "type": "waitForLight", "state": "confirm", "wfl_off_value": wfl_off_value, "wfl_on_value": wfl_on_value})

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
                self.warn("Could not finish wait for light calibration")
                return
            await self.calibrate_wfl(trie)
        print(f"Set value to: wfl {wfl_off_value}")


    async def __lcm_pump_async(self):
        while True:
            self.LCM.handle_timeout(0)
            await asyncio.sleep(0.01)
