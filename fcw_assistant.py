import Algorithms.fcw_algo as algo


class FCWAssistant:

    algorithm: algo.FCWAlgorithm

    def __init__(self, algorithm):
        if algorithm is None:
            raise Exception('No FCW algorithm was provided')
        else:
            self.algorithm = algorithm
            self.fcw_warning = None

    def evaluate_driving_situation(self, vehicle_info: dict):
        fcw_warning = self.algorithm.define_danger(vehicle_info=vehicle_info)
        fcw_warning.resolve_warning()

    def update_driver_info(self, driver_info: dict):
        self.algorithm.update_driver_dependent_constants(driver_info=driver_info)

    def update_environment_info(self, is_abs_on: bool, environment_info: dict):
        self.algorithm.update_environment_dependent_constants(is_abs_on=is_abs_on, environment_info=environment_info)
