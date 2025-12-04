import math
import time

from libstp.sensor_ir import IRSensor
import libstp.button as Button

class IRSensorCalibration:
    def __init__(self, buttonPort):
        Button.set_digital(buttonPort)

    def calibrateSensors(self, irsensors):
        print("Press the button when ready to scan the white values")
        Button.wait_for_button_press()
        print("Collecting data")
        t_end = time.time() + 10
        whiteValues = []
        blackValues = []
        while time.time() < t_end:
            for irsensor in irsensors:
                whiteValues.append(irsensor.read())
                time.sleep(0.01)

        print("Press the button when ready to scan the black values")
        Button.wait_for_button_press()
        print("Collecting data")
        t_end = time.time() + 10

        while time.time() < t_end:
            for irsensor in irsensors:
                blackValues.append(irsensor.read())
                time.sleep(0.01)

        print("Done now calibrating")

        if len(whiteValues) == 0 or len(blackValues) == 0:
            print("Got no Values for either White or Black")
            return

        for i in irsensors:
            self.calibrateSingle(i, blackValues, whiteValues)


    def __mean(self, v):
        return sum(v) / float(len(v))

    def __stddev(self, v, mean):
        return math.sqrt(sum((val - mean) ** 2 for val in v) / len(v))

    def calibrateSingle(self, irsensor, blackValues, whiteValues):
        whiteMean = self.__mean(whiteValues)
        blackMean = self.__mean(blackValues)

        if whiteMean >= blackMean:
            print(f"Calibration error: White values (mean: {whiteMean}) should be lower than black values (mean: {blackMean})",)
            print("This could indicate the sensors are positioned incorrectly or ambient light issues")
            return False

        whiteStdDev = max(1.0, self.__stddev(whiteValues, whiteMean))
        blackStdDev = max(1.0, self.__stddev(blackValues, blackMean))

        delta = blackMean - whiteMean

        if delta < 100.0:
            print(f"Insufficient contrast between white (mean: {whiteMean}) and black (mean: {blackMean}) values")
            print(f"Delta = {delta}. Recommended minimum delta is 100")
            return False

        if whiteStdDev > 0.2 * delta or blackStdDev > 0.2 * delta:
            print(f"High variance in readings: white stddev = {whiteStdDev}, black stddev = {blackStdDev}")
            print("This might indicate uneven lighting or unstable sensor position")

        #todo maybe remove factor later
        whiteThresh = (whiteMean + 0.5 * delta)
        blackThresh = (blackMean - 0.5 * delta)
        irsensor.setCalibration(whiteThresh, blackThresh)
        print(f"Calibration successful: white mean = {whiteMean}, black mean = {blackMean}")
        print(f"Thresholds set: white = {whiteThresh}, black = {blackThresh}")
        return True
