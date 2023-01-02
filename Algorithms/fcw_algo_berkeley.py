import math

import fcw_warnings
from Algorithms import fcw_algo


class FCWAlgorithmBerkeley(fcw_algo.FCWAlgorithm):

    __velocity: float
    __relative_velocity: float
    __deceleration: float
    __delay: float
    __distance: float
    __d_0: float
    __k1: float
    __k2: float
    __k3: float

    def __init__(self, d_0, k1, k2, k3):
        self.__d_0 = d_0
        self.__k1 = k1
        self.__k2 = k2
        self.__k3 = k3
        pass

    def define_dbr(self):

        d_br = self.__k1 * self.__velocity + self.__k2 * self.__relative_velocity + self.__k3

        return d_br

    def define_dw(self):

        d_w = 1 / 2 * ((self.__velocity**2) / self.__deceleration - ((self.__velocity - self.__relative_velocity) ** 2)
                       / self.__deceleration) + self.__velocity * self.__delay + self.__d_0

        return d_w

    def define_non_dim_warning_value(self):

        d_br = self.define_dbr()

        d_w = self.define_dw()

        w = (self.__distance - d_br) / (d_w - d_br)

        return w

    def define_danger(self, vehicle_info: dict) -> fcw_warnings.FCWWarning:

        w = self.define_non_dim_warning_value()

        situation_status = FCWWarningBerkeley(warning_level=w)

        return situation_status

    def update_algo_params(self, new_values: dict):
        self.__velocity = new_values.get('velocity')
        self.__relative_velocity = new_values.get('relative_velocity')
        self.__deceleration = new_values.get('deceleration')
        self.__delay = new_values.get('delay')
        self.__distance = new_values.get('distance')

        pass

    def update_driver_dependent_constants(self, driver_info: dict):
        pass

########################################################################################################################
# Warning Class for this type of implementation of FCW assistant
########################################################################################################################


class FCWWarningBerkeley(fcw_warnings.FCWWarning):

    __a = 0.0000000001

    def __init__(self, warning_level):
        self.warning_level = warning_level
        pass

    def resolve_warning(self):

        risk = self.warning_level

        # if self.warning_level < self.__a:
        #     situation = 'Dangerous'
        # elif self.warning_level <= 1:
        #     situation = 'Pay attention'
        # else:
        #     situation = 'Safe'

        if risk < self.__a:
            situation = 'Dangerous'
        elif risk <= 1:
            situation = 'Pay attention'
        else:
            situation = 'Safe'

        print('Situation status: ' + situation + '. Safety of situation ' + str(risk))

        pass
