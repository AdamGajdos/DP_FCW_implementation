from fcw_algo import *

class FCW_Algorithm_Friction(FCWAlgorithm):
    def __init__(self, **kwargs):
        pass

    def define_danger(self, vehicle_info: dict) -> fcw_warnings.FCWWarning:
        pass

    def update_driver_dependent_constants(self, driver_info: dict):
        pass

    def update_environment_dependent_constants(self, is_abs_on: bool, environment_info: dict):
        pass