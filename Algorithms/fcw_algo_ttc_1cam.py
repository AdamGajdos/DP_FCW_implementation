from Algorithms import fcw_algo
import fcw_warnings


class FCWAlgorithmTTC1Cam(fcw_algo.FCWAlgorithm):
    def define_danger(self, vehicle_info: dict) -> fcw_warnings.FCWWarning:
        pass

    def update_driver_dependent_constants(self, driver_info: dict):
        pass

    def update_environment_dependent_constants(self, is_abs_on: bool, environment_info: dict):
        pass

    def __init__(self, **kwargs):
        pass
