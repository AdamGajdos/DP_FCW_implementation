from abc import ABC, abstractmethod

import fcw_warnings


class FCWAlgorithm(ABC):

    @abstractmethod
    def __init__(self, **kwargs):
        pass

    @abstractmethod
    def define_danger(self, vehicle_info: dict) -> fcw_warnings.FCWWarning:
        pass

    @abstractmethod
    def update_driver_dependent_constants(self, driver_info: dict):
        pass

    # @abstractmethod
    # def update_environment_dependent_constants(self, is_abs_on: bool, environment_info: dict):
    #     pass

    @abstractmethod
    def update_algo_params(self, new_values: dict):
        pass
