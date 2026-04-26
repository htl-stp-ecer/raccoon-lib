from __future__ import annotations

import random

import lcm
from libstp_api import render_screen

from python.libstp_api.exlcm import screen_render_t


def handle_message(channel, data):
    msg = screen_render_t.decode(data)
    print("-" * 10)
    print(channel)
    print("MESSAGE RECEIVED OLLA:", msg.values_count)
    values = msg.entries
    for value in values:
        print(value.key)
        print(value.value)
    print("-" * 10)


lc = lcm.LCM()

render = render_screen.RenderScreen()
try:
    while True:
        i = random.randint(1, 100) <= 50

        if i:
            print("Request Time")
            if random.randint(1, 3) == 2:
                render.render_wait_for_light_screen({"Hallo": "hallo"})
            else:
                render.render_wait_for_light_screen({"Hallo": "hallo"})
except KeyboardInterrupt:
    print("Exiting...")
