import math
import fcw_warnings
from Algorithms import fcw_algo
import vehicle_imu


class FCWAlgorithmTTCBrakingDistance(fcw_algo.FCWAlgorithm):

    __road_adhesion_max: float
    __road_adhesion_min: float
    __t_r_max: float
    __t_r_min: float

    __brake_eff_max = 0.95
    __brake_eff_min = 0.85
    __t_p_max = 0.75
    __t_p_min = 0.3
    __critical_ttc = 1.5  # [s]
    __g = 9.80665
    __equivalent_mass_coefficient = 1.04  # gamma ... 1.03 - 1.05
    __air_density = 1.18  # rho ... kg/m^3 ... when temperature is 25C
    __aerodynamic_resistance_coefficient = 0.45  # C_d ... 0.15 - 0.5
    __rolling_resistance_coefficient = 0.015  # f_r ... 0.012 - 0.015

    def __init__(self):
        pass

    def define_braking_distance(self, brake_efficiency: float, road_adhesion: float, vehicle_info: dict):
        c_ae = (self.__air_density * vehicle_info.get('area') * self.__aerodynamic_resistance_coefficient) / 2.0

        if vehicle_info.get('steep').value == vehicle_imu.SteepSign['UPHILL'].value:
            sign = 1
        else:
            sign = -1

        braking_distance = (self.__equivalent_mass_coefficient * vehicle_info.get('weight')) / (2 * self.__g * c_ae)

        braking_distance *= math.log2(
            abs(
                1 + (c_ae * vehicle_info.get('velocity') ** 2) /
                (brake_efficiency * (road_adhesion + self.__rolling_resistance_coefficient) * vehicle_info.get('weight')
                 * math.cos(vehicle_info.get('angle')) +
                 sign * vehicle_info.get('weight') * math.sin(vehicle_info.get('angle')))
            )
        )

        return braking_distance

    def define_safety_braking_distances(self, vehicle_info: dict):
        d_b_min = self.define_braking_distance(road_adhesion=self.__road_adhesion_min,
                                               brake_efficiency=self.__brake_eff_min,
                                               vehicle_info=vehicle_info)

        d_b_max = self.define_braking_distance(road_adhesion=self.__road_adhesion_max,
                                               brake_efficiency=self.__brake_eff_max,
                                               vehicle_info=vehicle_info)

        d_r_min = self.__t_r_min * vehicle_info.get('velocity')
        d_r_max = self.__t_r_max * vehicle_info.get('velocity')

        d_p_min = self.__t_p_min * vehicle_info.get('velocity')
        d_p_max = self.__t_p_max * vehicle_info.get('velocity')

        d_s_min = d_b_min + d_r_min + d_p_min
        d_s_max = d_b_max + d_r_max + d_p_max

        return d_s_min, d_s_max

    def apply_rule(self, rule: int, ttc: float, velocity: float):

        situation_status = FCWWarningTtcBrakingDistance(situation_status='Safe')

        if rule == 1:
            if ttc < self.__critical_ttc:
                situation_status = FCWWarningTtcBrakingDistance(situation_status='Danger level 3')
            else:
                if velocity > 0:
                    situation_status = FCWWarningTtcBrakingDistance(situation_status='Danger level 1')
                else:
                    situation_status = FCWWarningTtcBrakingDistance(situation_status='Danger level 2')

        elif rule == 2:
            if ttc < self.__critical_ttc:
                situation_status = FCWWarningTtcBrakingDistance(situation_status='Danger level 2')
            else:
                if velocity > 0:
                    situation_status = FCWWarningTtcBrakingDistance(situation_status='Safe')
                else:
                    situation_status = FCWWarningTtcBrakingDistance(situation_status='Danger level 1')

        return situation_status

    def define_danger(self, vehicle_info: dict):

        ds_min, ds_max = self.define_safety_braking_distances(vehicle_info=vehicle_info)

        ttc = vehicle_info.get('distance') / vehicle_info.get('velocity')

        if vehicle_info.get('distance') < ds_min:
            situation_status = self.apply_rule(rule=1, ttc=ttc, velocity=vehicle_info.get('velocity'))
        elif vehicle_info.get('distance') < ds_max:
            situation_status = self.apply_rule(rule=2, ttc=ttc, velocity=vehicle_info.get('velocity'))
        else:
            situation_status = FCWWarningTtcBrakingDistance(situation_status='Safe')

        return situation_status

    def update_driver_dependent_constants(self, driver_info: dict):

        driver_age = driver_info.get('age')

        if 18 <= driver_age <= 33:
            self.__t_r_min = 0.74
            self.__t_r_max = 1.1
        elif 34 <= driver_age <= 47:
            self.__t_r_min = 0.75
            self.__t_r_max = 1.12
        elif 48 <= driver_age <= 57:
            self.__t_r_min = 0.77
            self.__t_r_max = 1.13
        elif 58 <= driver_age <= 70:
            self.__t_r_min = 0.78
            self.__t_r_max = 1.17
        else:
            self.__t_r_min = 0.8
            self.__t_r_max = 1.2

    def update_environment_dependent_constants(self, is_abs_on: bool, environment_info: dict):

        road_type = environment_info.get('road_info').road_type
        road_condition = environment_info.get('road_info').condition

        if is_abs_on:
            if road_type == vehicle_imu.RoadType.ASPHALT:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    self.__road_adhesion_min = 0.8
                    self.__road_adhesion_max = 0.9
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    self.__road_adhesion_min = 0.5
                    self.__road_adhesion_max = 0.7
            elif road_type == vehicle_imu.RoadType.CONCRETE:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    self.__road_adhesion_min = 0.8
                    self.__road_adhesion_max = 0.9
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    self.__road_adhesion_min = 0.8
                    self.__road_adhesion_max = 0.8
            elif road_type == vehicle_imu.RoadType.SNOW:
                self.__road_adhesion_min = 0.2
                self.__road_adhesion_max = 0.2
            elif road_type == vehicle_imu.RoadType.ICE:
                self.__road_adhesion_min = 0.1
                self.__road_adhesion_max = 0.1

        else:
            if road_type == vehicle_imu.RoadType.ASPHALT:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    self.__road_adhesion_min = 0.75
                    self.__road_adhesion_max = 0.75
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    self.__road_adhesion_min = 0.45
                    self.__road_adhesion_max = 0.6
            elif road_type == vehicle_imu.RoadType.CONCRETE:
                if road_condition == vehicle_imu.RoadCondition.DRY:
                    self.__road_adhesion_min = 0.75
                    self.__road_adhesion_max = 0.75
                elif road_condition == vehicle_imu.RoadCondition.WET:
                    self.__road_adhesion_min = 0.7
                    self.__road_adhesion_max = 0.7
            elif road_type == vehicle_imu.RoadType.SNOW:
                self.__road_adhesion_min = 0.15
                self.__road_adhesion_max = 0.15
            elif road_type == vehicle_imu.RoadType.ICE:
                self.__road_adhesion_min = 0.07
                self.__road_adhesion_max = 0.07

########################################################################################################################
# Warning Class for this type of implementation of FCW assistant
########################################################################################################################


class FCWWarningTtcBrakingDistance(fcw_warnings.FCWWarning):

    situation_status: str

    def __init__(self, situation_status):
        self.situation_status = situation_status

    def resolve_warning(self):
        print(self.situation_status)
