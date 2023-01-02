import vehicle_imu
from Algorithms import fcw_algo
import fcw_warnings


class FCWAlgorithmCustom(fcw_algo.FCWAlgorithm):

    __v: float                         # velocity of our vehicle
    __v_rel: float                     # relative velocity
    __a_max: float                     # max deceleration of our vehicle
    __d_0: float                       # headway offset
    __road_info: vehicle_imu.RoadInfo  # info about road type and condition
    __is_abs_on: bool                  # is ABS active

    __w_1 = 1.7           # weight for preferred warning distance in weighted avg.
    __w_2 = 1.3           # weight for the other warning distance in weighted avg.
    __tau_driver = 1      # delay of driver [s]
    __tau_system = 0.25   # delay of system [s]

    def __init__(self, d_0, a_max):

        self.__a_max = a_max
        self.__d_0 = d_0

        pass

    def define_friction_coef(self, is_abs_on, road_type, road_condition):

        road_adhesion_min = 1
        road_adhesion_max = 1

        if is_abs_on:
            if road_type == vehicle_imu.RoadType.ASPHALT:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    road_adhesion_min = 0.8
                    road_adhesion_max = 0.9
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    road_adhesion_min = 0.5
                    road_adhesion_max = 0.7
            elif road_type == vehicle_imu.RoadType.CONCRETE:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    road_adhesion_min = 0.8
                    road_adhesion_max = 0.9
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    road_adhesion_min = 0.8
                    road_adhesion_max = 0.8
            elif road_type == vehicle_imu.RoadType.SNOW:
                road_adhesion_min = 0.2
                road_adhesion_max = 0.2
            elif road_type == vehicle_imu.RoadType.ICE:
                road_adhesion_min = 0.1
                road_adhesion_max = 0.1

        else:
            if road_type == vehicle_imu.RoadType.ASPHALT:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    road_adhesion_min = 0.75
                    road_adhesion_max = 0.75
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    road_adhesion_min = 0.45
                    road_adhesion_max = 0.6
            elif road_type == vehicle_imu.RoadType.CONCRETE:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    road_adhesion_min = 0.75
                    road_adhesion_max = 0.75
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    road_adhesion_min = 0.7
                    road_adhesion_max = 0.7
            elif road_type == vehicle_imu.RoadType.SNOW:
                road_adhesion_min = 0.15
                road_adhesion_max = 0.15
            elif road_type == vehicle_imu.RoadType.ICE:
                road_adhesion_min = 0.07
                road_adhesion_max = 0.07

        friction_coef = (road_adhesion_min + road_adhesion_max) / 2

        return friction_coef

    def count_weighted_avg(self, relative_velocity, d_1, d_2):

        # Because different conservativeness of Honda and Mazda we will try to choose the right approach
        # according to actual situation. If relative velocity (v_leading - v_ours) is >= 0 then leading vehicle
        # is speeding/moving faster than we do, so conservative  approach is more suitable, but the other is valid too.
        # Because of that we consider both values, but with different weights.
        # Otherwise, when relative velocity < 0, then we would like to prefer non conservative approach.

        if relative_velocity >= 0:
            nominator = d_1 * self.__w_1 + d_2 * self.__w_2
        else:
            nominator = d_1 * self.__w_2 + d_2 * self.__w_1

        denominator = self.__w_1 + self.__w_2

        weighted_avg = nominator / denominator

        return weighted_avg

    def define_warning_distance(self):

        # SDA ; a_1 = a_2 = 5.88 m/s^2
        a_1_sda = 5.88
        a_2_sda = 5.88
        d_w_sda = self.__v * self.__tau_driver + self.__v / (2 * a_1_sda) - ((self.__v - self.__v_rel) ** 2) / (
                2 * a_2_sda)

        # WANG ; warning distance higher because of neglected traffic efficiency according
        # to work: "Vehicle forward collision warning algorithm based on road friction"
        # a_1 = a_max
        d_w_wang = self.__v * self.__tau_driver - self.__v_rel * self.__tau_system + (
                    (self.__v ** 2 - (self.__v - self.__v_rel) ** 2) / (2 * self.__a_max)) + self.__d_0

        # HONDA
        d_w_h = 2.2 * -self.__v_rel + 6.2

        d_w1 = d_w_h

        d_w2 = d_w_wang  # Try also sda, if results will be better

        d_w = self.count_weighted_avg(relative_velocity=self.__v_rel, d_1=d_w1, d_2=d_w2)

        return d_w

    def define_critical_braking_distance(self):

        # HONDA ; a_1 = a_2 = 7.8 m/s^2 ; tau_system = 0.5 s ; tau_braking_time =  1.5s
        a_1_h = 7.8
        a_2_h = 7.8
        tau_system_h = 0.5
        tau_braking_time_h = 1.5
        v_2 = self.__v - self.__v_rel
        ratio = v_2 / a_2_h

        if ratio < tau_braking_time_h:
            d_br_h = tau_braking_time_h * self.__v_rel + tau_system_h * tau_braking_time_h * a_1_h - (1 / 2) * a_1_h * (
                        tau_system_h ** 2)
        else:
            d_br_h = tau_braking_time_h * self.__v - (1 / 2) * a_1_h * ((tau_braking_time_h - tau_system_h) ** 2) - (
                    (v_2 ** 2) / (2 * a_2_h))

        # MAZDA ; a_1 = 6 m/s^2 ; a_2 = 8 m/s^2 ; tau_system = 0.1 s ; tau_driver = 0.6 s
        a_1_m = 6
        a_2_m = 8
        tau_system_m = 0.1
        tau_driver_m = 0.6
        d_br_m = (1 / 2) * ((self.__v ** 2) / a_1_m - ((self.__v - self.__v_rel) ** 2) / a_2_m) + \
                 self.__v * tau_system_m + self.__v_rel * tau_driver_m + self.__d_0

        # According to work "Development of a Collision Avoidance System - Berkeley" Mazda's way is more conservative
        # than Honda. Mazda tries to avoid even extreme cases on the road.
        d_br1 = d_br_m
        d_br2 = d_br_h

        road_type = self.__road_info.road_type
        road_condition = self.__road_info.condition

        d_br = self.count_weighted_avg(relative_velocity=self.__v_rel,
                                       d_1=d_br1, d_2=d_br2) * self.define_friction_coef(is_abs_on=self.__is_abs_on,
                                                                                         road_type=road_type,
                                                                                         road_condition=road_condition)

        return d_br

    def define_danger(self, vehicle_info: dict) -> fcw_warnings.FCWWarning:

        distance = vehicle_info.get('distance')

        d_w = self.define_warning_distance()

        d_br = self.define_critical_braking_distance()

        if distance > d_w:
            message_text = 'Safe.'
        elif d_br < distance <= d_w:
            message_text = 'Careful. Situation is getting dangerous.'
        else:  # distance <= d_br
            message_text = 'Applying automatic braking emergency system / Apply Brakes Now!'

        situation_status = FCWWarningCustom(situation_status=message_text)

        return situation_status

    def update_driver_dependent_constants(self, driver_info: dict):
        pass

    def update_environment_dependent_constants(self, new_values: dict):
        pass

    def update_algo_params(self, new_values: dict):
        self.__v = new_values.get('velocity')
        self.__v_rel = new_values.get('relative_velocity')
        self.__road_info = new_values.get('road_info')
        self.__is_abs_on = new_values.get('abs_on')
        pass


########################################################################################################################
# Warning Class for this type of implementation of FCW assistant
########################################################################################################################


class FCWWarningCustom(fcw_warnings.FCWWarning):
    situation_status: str

    def __init__(self, situation_status):
        self.situation_status = situation_status

    def resolve_warning(self):
        print(self.situation_status)
