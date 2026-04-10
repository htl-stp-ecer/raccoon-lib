from raccoon.hal import AnalogSensor, Motor, Servo

i = 0
sensor = Motor(0)
while True:
    print(sensor.read())
    # i+=1
    # Motor(0).set_speed(i)
    # Servo(0).set_position(i)