import random

import lcm

from libstp_api import render_screen
from libstp_api.exlcm import dict_t


def handle_message(channel, data):
    msg = dict_t.decode(data)
    print("-" * 10)
    print(channel)
    print("MESSAGE RECEIVED OLLA:", msg.values_count)
    values = msg.entries
    for value in values:
        print(value.key)
        print(value.value)
    print("-" * 10)


lc = lcm.LCM()
subscription = lc.subscribe("libstp/wait_for_light", handle_message)
subscriptiont = lc.subscribe("libstp/calibrate_sensors", handle_message)

try:
    while True:
        i = random.randint(1,100) <= 2

        if i:
            print("Request Time")
            if random.randint(1,3) == 2:
                render_screen.render_wait_for_light_screen({"Hallo" : "hallo"})
            else:
                render_screen.render_wait_for_light_screen({"Hallo" : "hallo"})
            lc.handle()
except KeyboardInterrupt:
    print("Exiting...")
