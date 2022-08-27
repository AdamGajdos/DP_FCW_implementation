import math

from fcw_algo import *

class FCW_Algorithm_TTC_1Cam(FCWAlgorithm):

    vehicle_imu: vehicle_imu.IMU

    def __init__(self, imu):
        self.vehicle_imu = imu

    def define_danger(self):

        c = self.vehicle_imu.acceleration * self.vehicle_imu.distance / (self.vehicle_imu.velocity ** 2)

        ttc = (-1 + math.sqrt(1 + 2 * c)) / c

        return