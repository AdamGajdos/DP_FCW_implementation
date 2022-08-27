import vehicle_driver
import math
import fcw_warnings
import Algorithms.fcw_algo as algo
import vehicle_imu


class FCWAlgorithmTTCBrakingDistance(algo.FCWAlgorithm):
    vehicle_IMU: vehicle_imu.IMU
    vehicle_driver: vehicle_driver.VehicleDriver

    road_adhesion_max: float
    road_adhesion_min: float
    brake_eff_max: float
    brake_eff_min: float
    t_r_max: float
    t_r_min: float
    t_p_max: float
    t_p_min: float

    critical_ttc: float  # [s]
    g: float
    equivalent_mass_coefficient: float  # gamma ... 1.03 - 1.05
    air_density: float  # rho ... kg/m^3 ... when temperature is 25C
    aerodynamic_resistance_coefficient: float  # C_d ... 0.15 - 0.5
    rolling_resistance_coefficient: float  # f_r ... 0.012 - 0.015

    def __init__(self, imu, driver):
        self.vehicle_IMU = imu
        self.vehicle_driver = driver

        self.set_constants()

        self.brake_eff_min = 0.85
        self.brake_eff_max = 0.95
        self.t_p_min = 0.3
        self.t_p_max = 0.75
        self.critical_ttc = 1.5
        self.g = 9.80665
        self.equivalent_mass_coefficient = 1.04
        self.air_density = 1.18
        self.aerodynamic_resistance_coefficient = 0.45
        self.rolling_resistance_coefficient = 0.015

    def update_imu(self, new_imu: vehicle_imu.IMU):
        self.vehicle_IMU = vehicle_imu.IMU(
            weight=new_imu.weight,
            velocity=new_imu.velocity,
            acceleration=new_imu.acceleration,
            area=new_imu.area,
            steep=new_imu.steep,
            angle=new_imu.angle,
            road_info=new_imu.road_info,
            distance=new_imu.distance,
            has_abs=new_imu.ABS)

    def set_constants(self):

        imu = self.vehicle_IMU
        age = self.vehicle_driver.age

        if imu.ABS:
            if imu.road_info.type == vehicle_imu.RoadType.ASPHALT:
                if imu.road_info.condition == vehicle_imu.RoadCondition.DRY:
                    self.road_adhesion_min = 0.8
                    self.road_adhesion_max = 0.9
                elif imu.road_info.condition == vehicle_imu.RoadCondition.WET:
                    self.road_adhesion_min = 0.5
                    self.road_adhesion_max = 0.7
            elif imu.road_info.type == vehicle_imu.RoadType.CONCRETE:
                if imu.road_info.condition == vehicle_imu.RoadCondition.DRY:
                    self.road_adhesion_min = 0.8
                    self.road_adhesion_max = 0.9
                elif imu.road_info.condition == vehicle_imu.RoadCondition.WET:
                    self.road_adhesion_min = 0.8
                    self.road_adhesion_max = 0.8
            elif imu.road_info.type == vehicle_imu.RoadType.SNOW:
                self.road_adhesion_min = 0.2
                self.road_adhesion_max = 0.2
            elif imu.road_info.type == vehicle_imu.RoadType.ICE:
                self.road_adhesion_min = 0.1
                self.road_adhesion_max = 0.1

        else:
            if imu.road_info.type == vehicle_imu.RoadType.ASPHALT:
                if imu.road_info.condition == vehicle_imu.RoadCondition.DRY:
                    self.road_adhesion_min = 0.75
                    self.road_adhesion_max = 0.75
                elif imu.road_info.condition == vehicle_imu.RoadCondition.WET:
                    self.road_adhesion_min = 0.45
                    self.road_adhesion_max = 0.6
            elif imu.road_info.type == vehicle_imu.RoadType.CONCRETE:
                if imu.road_info.condition == vehicle_imu.RoadCondition.DRY:
                    self.road_adhesion_min = 0.75
                    self.road_adhesion_max = 0.75
                elif imu.road_info.condition == vehicle_imu.RoadCondition.WET:
                    self.road_adhesion_min = 0.7
                    self.road_adhesion_max = 0.7
            elif imu.road_info.type == vehicle_imu.RoadType.SNOW:
                self.road_adhesion_min = 0.15
                self.road_adhesion_max = 0.15
            elif imu.road_info.type == vehicle_imu.RoadType.ICE:
                self.road_adhesion_min = 0.07
                self.road_adhesion_max = 0.07

        self.update_driver_dependent_constants(driver_age=age)

    def define_braking_distance(self, brake_efficiency, road_adhesion):
        C_ae = (self.air_density * self.vehicle_IMU.area * self.aerodynamic_resistance_coefficient) / 2.0

        if self.vehicle_IMU.steep.value == vehicle_imu.SteepSign['UPHILL'].value:
            sign = 1
        else:
            sign = -1

        braking_distance = (self.equivalent_mass_coefficient * self.vehicle_IMU.weight) / (2 * self.g * C_ae)

        braking_distance *= math.log2(
            abs(
                1 + (C_ae * self.vehicle_IMU.velocity ** 2) /
                (brake_efficiency * (road_adhesion + self.rolling_resistance_coefficient) * self.vehicle_IMU.weight
                 * math.cos(self.vehicle_IMU.angle) + sign * self.vehicle_IMU.weight * math.sin(self.vehicle_IMU.angle))
            )
        )

        return braking_distance

    def define_ttc(self):

        ttc = self.vehicle_IMU.distance / self.vehicle_IMU.velocity

        return ttc

    def define_safety_braking_distances(self):
        D_b_min = self.define_braking_distance(road_adhesion=self.road_adhesion_min,
                                               brake_efficiency=self.brake_eff_min)

        D_b_max = self.define_braking_distance(road_adhesion=self.road_adhesion_max,
                                               brake_efficiency=self.brake_eff_max)

        D_r_min = self.t_r_min * self.vehicle_IMU.velocity
        D_r_max = self.t_r_max * self.vehicle_IMU.velocity

        D_p_min = self.t_p_min * self.vehicle_IMU.velocity
        D_p_max = self.t_p_max * self.vehicle_IMU.velocity

        D_s_min = D_b_min + D_r_min + D_p_min
        D_s_max = D_b_max + D_r_max + D_p_max

        return D_s_min, D_s_max

    def apply_rule(self, rule: int):

        situation_status = fcw_warnings.DangerLevel.SAFE

        ttc = self.define_ttc()

        if rule == 1:
            if ttc < self.critical_ttc:
                situation_status = fcw_warnings.DangerLevel.LEVEL3
            else:
                if self.vehicle_IMU.velocity > 0:
                    situation_status = fcw_warnings.DangerLevel.LEVEL1
                else:
                    situation_status = fcw_warnings.DangerLevel.LEVEL2

        elif rule == 2:
            if ttc < self.critical_ttc:
                situation_status = fcw_warnings.DangerLevel.LEVEL2
            else:
                if self.vehicle_IMU.velocity > 0:
                    situation_status = fcw_warnings.DangerLevel.SAFE
                else:
                    situation_status = fcw_warnings.DangerLevel.LEVEL1

        return situation_status

    def define_danger(self):

        ds_min, ds_max = self.define_safety_braking_distances()

        if self.vehicle_IMU.distance < ds_min:
            situation_status = self.apply_rule(rule=1)
        elif self.vehicle_IMU.distance < ds_max:
            situation_status = self.apply_rule(rule=2)
        else:
            situation_status = fcw_warnings.DangerLevel.SAFE

        return situation_status

    def update_driver_dependent_constants(self, driver_age):
        if 18 <= driver_age <= 33:
            self.t_r_min = 0.74
            self.t_r_max = 1.1
        elif 34 <= driver_age <= 47:
            self.t_r_min = 0.75
            self.t_r_max = 1.12
        elif 48 <= driver_age <= 57:
            self.t_r_min = 0.77
            self.t_r_max = 1.13
        elif 58 <= driver_age <= 70:
            self.t_r_min = 0.78
            self.t_r_max = 1.17
        else:
            self.t_r_min = 0.8
            self.t_r_max = 1.2
