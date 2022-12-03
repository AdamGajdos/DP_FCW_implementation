import math
import random

import scipy.integrate
from scipy.integrate import dblquad
import fcw_warnings
from Algorithms import fcw_algo


#  This algorithm will be removed
#  Margins for evaluation of situation severity are not defined in papers.
#  Actual values are chosen based on others study independent material.
class FCWAlgoFriction(fcw_algo.FCWAlgorithm):

    __v_1: float  # velocity of our vehicle
    __v_2: float  # velocity of leading vehicle
    __delta: float  # amplitude setting
    __omega: float  # regulation frequency
    __d_0: float  # safe stopping distance
    __g = 9.81
    __a_2: float  # acceleration of leading vehicle
    __tau_1: float  # system delay
    __tau_2: float  # driver delay
    __b: float  # TODO
    __s: float  # TODO
    __epsilon = math.pow(10, -10)
    __wheel_radius: int  # wheel radius

    def __init__(self, d_0, tau_1, tau_2, s):
        self.__d_0 = d_0
        self.__tau_1 = tau_1
        self.__tau_2 = tau_2
        self.__s = s
        pass

#    def __function_omega:  #conversion from formula: v = r * omega' ... r is wheel radius ... omega' is angular velocity
#        return self.__v_1 / vehicle.wheel_radius

    # t - time
    def leading_vehicle_is_static(self, integration: float):

        #function_omega = lambda omega, t: random.randrange(19, 126)  # 19 - 126 rad/s -- based on wheel speed and pavement
        #
        #function_to_integrate = lambda t: (self.__a_2 * self.__s ** 2 + self.__b * self.__s)*self.__g - \
        #                                  self.__delta / 2 + (self.__delta/2) * math.cos(function_omega(t))
        #
        # upper_limit_outer = 0
        # lower_limit_outer = self.__v_1 / ((self.__a_2 * (self.__s ** 2) + self.__b * self.__s) * self.__g)
        #
        # upper_limit_inner = self.__v_1 / ((self.__a_2 * (self.__s ** 2) + self.__b * self.__s) * self.__g)
        # lower_limit_inner = 0
        #
        # integration = dblquad(function_to_integrate,
        #                         lower_limit_outer, upper_limit_outer,
        #                         lower_limit_inner, upper_limit_inner) ... take integration[0]

        d_w1 = self.__v_1 * self.__tau_1 + self.__v_1 * self.__tau_2 - \
                      (self.__a_2 * self.__s**2 + self.__b * self.__s)/6 * self.__g * (self.__tau_2 ** 2) + \
                      integration + self.__d_0

        d_br1 = self.__v_1 * self.__tau_2 - (self.__a_2 * self.__s**2 + self.__b * self.__s) / 6 * \
                       self.__g * (self.__tau_2 ** 2) + integration + self.__d_0

        return d_w1, d_br1

    def leading_vehicle_is_braking(self, integration: float):

        # function_omega = lambda omega, t: random.randrange(19, 126)  # 19 - 126 rad/s -- based on wheel speed,  pavement
        #
        # function_to_integrate = lambda t: (self.__a_2 * self.__s ** 2 + self.__b * self.__s) * self.__g - \
        #                                   self.__delta / 2 + (self.__delta / 2) * math.cos(function_omega(t))
        #
        # upper_limit_outer = 0
        # lower_limit_outer = self.__v_1 / ((self.__a_2 * (self.__s ** 2) + self.__b * self.__s) * self.__g)
        #
        # upper_limit_inner = self.__v_1 / ((self.__a_2 * (self.__s ** 2) + self.__b * self.__s) * self.__g)
        # lower_limit_inner = 0
        #
        # integration = dblquad(function_to_integrate,
        #                                       lower_limit_outer, upper_limit_outer,
        #                                       lower_limit_inner, upper_limit_inner)

        d_w1 = (self.__v_1 - self.__v_2) * (self.__tau_1 + self.__tau_2) - self.__g * (self.__tau_2 ** 2) * \
               (self.__a_2 * self.__s**2 + self.__b * self.__s)/6 - \
               (self.__v_1 * self.__v_2)/((self.__a_2 * self.__s**2 + self.__b * self.__s) * self.__g) + \
               integration + self.__d_0

        d_br1 = (self.__v_1-self.__v_2) * self.__tau_2 - self.__v_2*self.__tau_1 - (self.__g*self.__tau_2**2) * \
                (self.__a_2 * self.__s**2 + self.__b * self.__s)/6 - \
                (self.__v_1 * self.__v_2)/((self.__a_2 * self.__s**2 + self.__b * self.__s) * self.__g) + \
                integration + self.__d_0

        return d_w1, d_br1

    def leading_vehicle_is_moving_forward(self, integration: float):

        # function_omega = lambda omega, t: random.randrange(19, 126)  # 19 - 126 rad/s -- based on wheel speed,  pavement
        #
        # function_to_integrate = lambda t: (self.__a_2 * self.__s ** 2 + self.__b * self.__s) * self.__g - \
        #                                   self.__delta / 2 + (self.__delta / 2) * math.cos(function_omega(t))
        #
        # upper_limit_outer = self.__v_2 / ((self.__a_2 * self.__s**2 + self.__b * self.__s) * self.__g)
        # lower_limit_outer = self.__v_1 / ((self.__a_2 * (self.__s ** 2) + self.__b * self.__s) * self.__g)
        #
        # upper_limit_inner = (self.__v_2 - self.__v_1) / ((self.__a_2 * (self.__s ** 2) + self.__b * self.__s) * self.__g)
        # lower_limit_inner = 0
        #
        # integration = dblquad(function_to_integrate,
        #                                       lower_limit_outer, upper_limit_outer,
        #                                       lower_limit_inner, upper_limit_inner)

        d_w1 = (self.__v_1 - self.__v_2) * (self.__tau_1 + self.__tau_2) - self.__g * (self.__tau_2 ** 2) * \
               (self.__a_2 * self.__s ** 2 + self.__b * self.__s) / 6 - \
               (self.__v_1 - self.__v_2)/((self.__a_2 * self.__s**2 + self.__b * self.__s) * self.__g) * self.__v_2 + \
               integration + self.__d_0

        d_br1 = (self.__v_1 - self.__v_2) * self.__tau_2 - self.__v_2*self.__tau_1 - self.__g * (self.__tau_2 ** 2) * \
                (self.__a_2 * self.__s ** 2 + self.__b * self.__s) / 6 - (self.__v_1 - self.__v_2) * self.__v_2 / \
                ((self.__a_2 * self.__s ** 2 + self.__b * self.__s) * self.__g) + integration + self.__d_0

        return d_w1, d_br1

    def identify_leading_vehicle_state(self, vehicle_info: dict):

        result = 'Static'

        if 0 + self.__epsilon > vehicle_info.get('velocity') > 0 - self.__epsilon:
            if 0 + self.__epsilon > vehicle_info.get('acceleration') > 0 - self.__epsilon:
                result = 'Static'
        elif vehicle_info.get('velocity') >= 0 + self.__epsilon:
            if vehicle_info.get('acceleration') >= 0:
                result = 'Forward'
            else:
                result = 'Breaking'

        return result

    def define_danger(self, vehicle_info: dict) -> fcw_warnings.FCWWarning:

        vehicle_state = self.identify_leading_vehicle_state(vehicle_info)

        if vehicle_state == 'Static':
            d_w, d_br = self.leading_vehicle_is_static()

        elif vehicle_state == 'Breaking':
            d_w, d_br = self.leading_vehicle_is_braking()

        elif vehicle_state == 'Forward':
           d_w, d_br = self.leading_vehicle_is_moving_forward()

        else:
            raise Exception

        warning_message = ''
        if vehicle_info.get('distance') <= d_w:
            if vehicle_info.get('distance') <= d_br:
                warning_message = 'Applying automatic brakes!'
            else:
                warning_message = 'Be careful!'

        return FCWWarningFriction(message=warning_message)

    def update_driver_dependent_constants(self, driver_info: dict):
        pass

    def update_algo_params(self, new_values: dict):
        self.__v_1 = new_values.get('velocity')
        self.__v_2 = new_values.get('relative_velocity') + self.__v_1
        self.__a_2 = new_values.get('TODO')  # TODO ... a2 is NOT acceleration
        self.__delta = new_values.get('TODO')  # TODO ... adjusting amplitude
        self.__wheel_radius = new_values.get('wheel_radius')
        # __b: float  # TODO
        # __s: float  # TODO

        pass


class FCWWarningFriction(fcw_warnings.FCWWarning):

    message: str

    def __init__(self, message: str):
        self.message = message
        pass

    def resolve_warning(self):

        print(self.message)

        pass
