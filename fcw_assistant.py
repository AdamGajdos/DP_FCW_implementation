import Algorithms.fcw_algo as algo
import fcw_warnings
import vehicle_imu


class FCWAssistant:

    algorithm: algo.FCWAlgorithm

    def __init__(self, algorithm):
        if algorithm is None:
            raise Exception('No FCW algorithm was provided')
        else:
            self.algorithm = algorithm
            self.fcw_warning = None

    def evaluate_driving_situation(self, vehicle_info: dict):
        fcw_warning = self.algorithm.define_danger()
        self.__present_situation_status(fcw_warning)

    @staticmethod
    def __present_situation_status(fcw_warning: fcw_warnings.FCWWarning):
        pass

    def update_driver_info(self, driver_info):
        self.algorithm.update_driver_dependent_constants(driver_info)

    def update_environment_info(self, environment_info):
        self.algorithm.update_environment_dependent_constants(environment_info)