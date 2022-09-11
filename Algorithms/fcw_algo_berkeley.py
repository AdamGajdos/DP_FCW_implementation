import math

from Algorithms import fcw_algo
import fcw_warnings


class FCWAlgorithmBerkeley(fcw_algo.FCWAlgorithm):
    def __init__(self, d_0, k1, k2, k3):
        self.__d_0 = d_0
        self.__k1 = k1
        self.__k2 = k2
        self.__k3 = k3
        pass

    def define_dbr(self, velocity, relative_velocity):

        d_br = self.__k1 * velocity + self.__k2 * relative_velocity + self.__k3

        return d_br

    def define_dw(self, velocity, relative_velocity, alfa, tau):

        d_w = 1/2 * ((velocity**2) / alfa - ((velocity - relative_velocity)**2)/alfa) + velocity * tau + self.__d_0

        return d_w

    def define_non_dim_warning_value(self, vehicle_info: dict):

        velocity = vehicle_info.get('velocity')

        relative_velocity = vehicle_info.get('relative_velocity')

        alfa = vehicle_info.get('alfa')

        tau = vehicle_info.get('tau')

        d = vehicle_info.get('distance')

        d_br = self.define_dbr(velocity=velocity, relative_velocity=relative_velocity)

        d_w = self.define_dw(velocity=velocity, relative_velocity=relative_velocity, alfa=alfa, tau=tau)

        w = (d - d_br) / (d_w - d_br)

        return w

    def define_danger(self, vehicle_info: dict) -> fcw_warnings.FCWWarning:

        w = self.define_non_dim_warning_value(vehicle_info=vehicle_info)

        situation_status = FCWWarningBerkeley(warning_level=w)

        return situation_status

    def update_driver_dependent_constants(self, driver_info: dict):
        pass

    def update_environment_dependent_constants(self, is_abs_on: bool, environment_info: dict):
        pass

########################################################################################################################
# Warning Class for this type of implementation of FCW assistant
########################################################################################################################


class FCWWarningBerkeley(fcw_warnings.FCWWarning):

    __a = math.pow(10, -15)

    def __init__(self, warning_level):
        self.warning_level = warning_level
        pass

    def resolve_warning(self):

        risk = 1 / self.warning_level

        if self.warning_level < self.__a:
            situation = 'Dangerous'
        elif self.warning_level <= 1:
            situation = 'Pay attention'
        else:
            situation = 'Safe'

        print('Situation status: ' + situation + '. Risk of crash ' + str(risk) + ' %')

        pass
