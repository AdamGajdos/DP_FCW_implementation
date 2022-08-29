import math
from enum import Enum


class SteepSign(Enum):
    UPHILL = 1
    DOWNHILL = -1


class RoadType(Enum):
    ASPHALT = 0
    CONCRETE = 1
    SNOW = 2
    ICE = 3


class RoadCondition(Enum):
    WET = 0
    DRY = 1


class RoadInfo:
    road_type: RoadType
    condition: RoadCondition

    def __init__(self, road_type: RoadType, condition: RoadCondition):
        self.road_type = road_type
        self.condition = condition


class IMU:
    velocity: float         # [m/s]
    acceleration: float     # [m/s^2]
    angle: float            # [°] ... vertical angle of steeping on road
    distance: float         # [m] ... distance between vehicle and frontal obstacle

    road_info: RoadInfo
    steep: SteepSign

    def __init__(self, velocity, acceleration, steep, angle, road_info, distance):

        self.velocity = velocity
        self.acceleration = acceleration
        self.steep = steep
        self.angle = angle
        self.road_info = road_info
        self.distance = distance
