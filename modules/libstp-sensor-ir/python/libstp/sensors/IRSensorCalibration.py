import time
import numpy as np
from sklearn.cluster import KMeans

from libstp.sensor_ir import IRSensor
import libstp.button as Button

class IRSensorCalibration:
    def __init__(self, buttonPort):
        Button.set_digital(buttonPort)

    def calibrateSensors(self, irsensors):
        print("Press the button when ready to scan the values")
        Button.wait_for_button_press()
        values = self._collectValues(irsensors, 7)

        if not values:
            print("Got no values for WHITE or BLACK")
            return

        for sensor in irsensors:
            self.calibrateSingle(sensor, values)

    def _collectValues(self, irsensors, duration=5):
        print("Collecting Values")
        t_end = time.time() + duration
        values = []
        while time.time() < t_end:
            for sensor in irsensors:
                values.append(sensor.read())
                time.sleep(0.01)
        return values

    def calibrateSingle(self, irsensor, values):
        all_values = np.array(values).reshape(-1, 1)
        kmeans = KMeans(n_clusters=2, n_init=10, random_state=42)
        kmeans.fit(all_values)
        centers = sorted(kmeans.cluster_centers_.flatten())
        whiteThresh = centers[0]
        blackThresh = centers[1]
        irsensor.setCalibration(blackThresh, whiteThresh)
        print(f"White Threshold: {whiteThresh}, Black Threshold: {blackThresh}")
        return True
