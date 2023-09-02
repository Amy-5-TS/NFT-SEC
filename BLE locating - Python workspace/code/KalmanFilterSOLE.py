# python 3.x
import json
import random
import time
import matplotlib.pyplot as plt 
import math


class KalmanFilter:

    cov = float('nan')
    x = float('nan')

    def __init__(self, R, Q):
        """
        Constructor
        :param R: Process Noise
        :param Q: Measurement Noise
        """
        self.A = 1
        self.B = 0
        self.C = 1

        self.R = R
        self.Q = Q

    def filter(self, measurement):
        """
        Filters a measurement
        :param measurement: The measurement value to be filtered
        :return: The filtered value
        """
        u = 0
        if math.isnan(self.x):
            self.x = (1 / self.C) * measurement
            self.cov = (1 / self.C) * self.Q * (1 / self.C)
        else:
            predX = (self.A * self.x) + (self.B * u)
            predCov = ((self.A * self.cov) * self.A) + self.R

            # Kalman Gain
            K = predCov * self.C * (1 / ((self.C * predCov * self.C) + self.Q))

            # Correction
            self.x = predX + K * (measurement - (self.C * predX))
            self.cov = predCov - (K * self.C * predCov)

        return self.x

    def last_measurement(self):
        """
        Returns the last measurement fed into the filter
        :return: The last measurement fed into the filter
        """
        return self.x

    def set_measurement_noise(self, noise):
        """
        Sets measurement noise
        :param noise: The new measurement noise
        """
        self.Q = noise

    def set_process_noise(self, noise):
        """
        Sets process noise
        :param noise: The new process noise
        """
        self.R = noise


def run():
    test = KalmanFilter(0.008, 0.1)
    Kalfin=99
    kal=[]
    f = open("0.3m.txt" , "r")
    for line in f.readlines():
        if "-" in line:
            start = line.find("-") 
            end = line.find('-') + 3
            kal.append(int(line[start:end]))
    for x in kal:
        Kalfin=float(test.filter(x))
    f.close()
    fin = open("KalResult.txt" , "a")
    fin.write(str(Kalfin))
    fin.write("\n")
    fin.close()

if __name__ == '__main__':
    run()