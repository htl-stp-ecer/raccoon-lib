from libstp.hal import AnalogSensor, Motor, Servo

i = 0
sensor = AnalogSensor(0)
while True:
    print(sensor.read())
    # i+=1
    # Motor(0).set_speed(i)
    # Servo(0).set_position(i)