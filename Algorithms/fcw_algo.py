from abc import ABC, abstractmethod
import vehicle_imu


class FCWAlgorithm(ABC):

    @abstractmethod
    def __init__(self, **kwargs):
        pass

    @abstractmethod
    def define_danger(self):
        pass

    @abstractmethod
    def update_imu(self, imu: vehicle_imu.IMU):
        pass

    @abstractmethod
    def update_driver_dependent_constants(self, **driver_info):
        pass
