from typing import Any, Dict

from libstp_api.exlcm.dict_t import dict_t
from libstp_api.exlcm.key_value_t import key_value_t

import lcm
LCM = lcm.LCM()

def __send_screen_render_request_to_lcm(
        screen_name: str,
        data: Dict[str, Any]):
    msg = dict_t()
    msg.values_count = len(data)
    msg.entries = []

    for key, value in data.items():
        val = key_value_t()
        val.key = key
        val.value = value
        msg.entries.append(val)

    LCM.publish("libstp/" + screen_name, msg.encode())


def render_wait_for_light_screen(
        data: Dict[str, Any],
):
    __send_screen_render_request_to_lcm("wait_for_light", data)

def render_calibrate_sensors_screen(
        data: Dict[str, Any],
):
    __send_screen_render_request_to_lcm("calibrate_sensors", data)
