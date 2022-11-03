import math
import Algorithms.fcw_algo_ttc_braking_distance as fcw_algo_def
import fcw_assistant as fcw
import location
import vehicle_driver
import vehicle_imu


class Vehicle:

    weight: float                          # [kg]
    area: float                            # [cm^2] ... front area of vehicle
    max_speed: float                       # [m/s]
    name: str                              # identification of the car
    is_running: bool                       # is engine running
    is_braking: bool                       # did driver proceeded braking maneuver
    has_abs: bool                          # is vehicle equipped with abs
    abs_on: bool                           # is abs active
    wheel_radius: int                      # wheel radius in inches

    # direction: vector_custom.Vector        # [x,y,z] ... where car is heading
    position: location.Location            # longitude, latitude ... actual location of car

    driver: vehicle_driver
    imu: vehicle_imu

    fcw_assistant: fcw.FCWAssistant
    is_fcw_on: bool                       # is fcw assistant running

    def __init__(self, weight, area, max_speed, name, has_abs, fcw_assistant):
        self.name = name
        self.weight = weight
        self.area = area
        self.max_speed = max_speed
        self.is_running = False
        self.is_braking = False
        self.has_abs = has_abs
        self.abs_on = True if has_abs else False
        self.wheel_radius = 15

        # self.direction = vector_custom.Vector(x=0, y=0, z=0)
        self.position = location.Location(longitude=0.0, latitude=0.0)

        self.imu = self.__init_imu()

        self.driver = None

        self.fcw_assistant = fcw.FCWAssistant(algorithm=fcw_algo_def.FCWAlgorithmTTCBrakingDistance()) \
            if fcw_assistant is None else fcw_assistant
        self.is_fcw_on = True

        pass

    @staticmethod
    def __init_imu():

        # default assumption
        road_info = vehicle_imu.RoadInfo(
            road_type=vehicle_imu.RoadType.ASPHALT,
            condition=vehicle_imu.RoadCondition.DRY
        )

        steep = vehicle_imu.SteepSign.UPHILL

        imu = vehicle_imu.IMU(
            distance=math.inf,
            acceleration=0,
            deceleration=0,
            velocity=0,
            angle=0,
            steep=steep,
            road_info=road_info,
            delay=math.inf,
            relative_velocity=0
        )

        return imu

    def start_engine(self, driver_age):

        self.driver = vehicle_driver.VehicleDriver(age=driver_age)

        if self.driver is not None:

            driver_info = {'age': driver_age}

            self.fcw_assistant.update_driver_info(driver_info=driver_info)
            self.is_running = True

    def is_moving(self):

        is_moving = self.imu.velocity != 0 or self.imu.acceleration != 0   # if acceleration is constant

        return is_moving

    def fcw_switch(self):
        self.is_fcw_on = True if not self.is_fcw_on else False

    def abs_switch(self):
        self.abs_on = True if not self.has_abs else False

    def update_imu(self, distance, acceleration, deceleration, velocity, angle, road_info, steep, delay):
        self.imu.distance = distance
        self.imu.acceleration = acceleration
        self.imu.deceleration = deceleration
        self.imu.velocity = velocity
        self.imu.angle = angle
        self.imu.road_info = road_info
        self.imu.steep = steep
        self.imu.delay = delay

    def get_vehicle_info(self) -> dict:
        vehicle_info = {
            'velocity': self.imu.velocity,
            'acceleration': self.imu.acceleration,
            'deceleration': self.imu.deceleration,
            'relative_velocity': self.imu.relative_velocity,
            'weight': self.weight,
            'area': self.area,
            'abs_on': self.abs_on,
            'angle': self.imu.angle,
            'steep': self.imu.steep,
            'road_info': self.imu.road_info,
            'distance': self.imu.distance,
            'fcw_on': self.is_fcw_on,
            'delay': self.imu.delay,
            'wheel_radius': self.wheel_radius
        }

        return vehicle_info

    def move_vehicle(self, new_position: location.Location):

        # new_direction = vector_custom.Vector(
        #     x=new_position.x - self.position.x,
        #     y=new_position.y - self.position.y,
        #     z=new_position.z - self.position.z
        # )

        # self.direction = new_direction

        # environment_info = {
        #     'road_info': self.imu.road_info
        # }

        # self.fcw_assistant.update_environment_info(is_abs_on=self.abs_on, environment_info=environment_info)

        self.fcw_assistant.update_algo_params(self.get_vehicle_info())

        self.fcw_assistant.evaluate_driving_situation(vehicle_info=self.get_vehicle_info())

        self.position = new_position
