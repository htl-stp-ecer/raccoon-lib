from libstp.hal import Motor, Servo

i = 0
while True:
    i+=1
    Motor(0).set_speed(i)
    Servo(0).set_position(i)