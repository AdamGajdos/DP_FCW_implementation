import Algorithms.fcw_algo as alg
import Algorithms.fcw_algo_ttc_braking_distance
import vehicle_driver
import vehicle_imu
import fcw_warnings
import random


def resolve_danger(danger_level: fcw_warnings.DangerLevel):

    if danger_level != fcw_warnings.DangerLevel.SAFE:
        print(danger_level.name)

    pass

class FCW_Assistant:
    algorithm: alg.FCWAlgorithm

    def __init__(self, fcw_algo: alg.FCWAlgorithm):
        self.algorithm = fcw_algo


    def run(self):

        is_running = True

        imu = vehicle_imu.IMU(
            weight=1620,  # [kg]
            velocity=60.01,  # [m/s]
            acceleration=6.12,  # [m/s^2]
            area=1500,  # [cm^2]
            steep=vehicle_imu.SteepSign.UPHILL,  # +..1, -..-1
            angle=0,  # [C]
            road_info=vehicle_imu.RoadInfo(
                road_type=vehicle_imu.RoadType.ASPHALT,
                condition=vehicle_imu.RoadCondition.DRY
            ),
            distance=400,  # [m]
            has_abs=True
        )

        max_speed = 250
        max_acceleration = 10
        sign = 1
        max_distance = 200
        steep = vehicle_imu.SteepSign.UPHILL

        iterations = 0

        while is_running:

            iterations += 1

            print(iterations)

            danger = self.algorithm.define_danger()

            resolve_danger(danger)

            if danger == fcw_warnings.DangerLevel.LEVEL3:
                is_running = False

            imu.velocity = random.random() * max_speed
            imu.acceleration = random.random() * max_acceleration * sign
            imu.distance = random.random() * max_distance
            imu.angle = random.random() * 90

            if random.random() >= 0.5:
                steep = vehicle_imu.SteepSign.UPHILL
            else:
                steep = vehicle_imu.SteepSign.DOWNHILL

            imu.steep = steep

            self.algorithm.update_imu(new_imu=imu)

imu = vehicle_imu.IMU(
            weight=1620,  # [kg]
            velocity=60.01,  # [m/s]
            acceleration=6.12,  # [m/s^2]
            area=1500,  # [cm^2]
            steep=vehicle_imu.SteepSign.UPHILL,  # +..1, -..-1
            angle=0,  # [C]
            road_info=vehicle_imu.RoadInfo(
                road_type=vehicle_imu.RoadType.ASPHALT,
                condition=vehicle_imu.RoadCondition.DRY
            ),
            distance=400,  # [m]
            has_abs=True
        )

driver = vehicle_driver.VehicleDriver()
driver.age = 19

algorithm = Algorithms.fcw_algo_ttc_braking_distance.FCWAlgorithmTTCBrakingDistance(imu=imu,
                                                                                    driver=driver)
fcw_assistant = FCW_Assistant(fcw_algo=algorithm)

fcw_assistant.run()