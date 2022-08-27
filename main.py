import json
import Algorithms.fcw_factory
import vector_custom
import vehicle
import vehicle_driver
import vehicle_imu


class _RoadPoint:
    position: vector_custom.Vector
    velocity: float
    road_info: vehicle_imu.RoadInfo
    distance: float
    acceleration: float
    steep: vehicle_imu.SteepSign
    angle: float

    def __init__(self, position, velocity, road_info, distance, acceleration, steep, angle):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.distance = distance
        self.road_info = road_info
        self.steep = steep
        self.angle = angle


def load_data(file_path: str):
    try:
        f = open(file_path, mode='r')
        loaded_data = json.load(f)

        return loaded_data

    except OSError:

        print('File could not be opened and read')

        return None


class FCWSimulation:
    vehicles = []
    vehicle_drivers = []
    road = []
    fcw_factory = Algorithms.fcw_factory.FCWFactory()

    def __init__(self):
        self.__prepare_vehicles()
        self.__prepare_vehicle_drivers()
        self.__prepare_road()

    def __prepare_vehicles(self):

        vehicles_data = load_data('SimulationData/vehicle_data.json')

        if vehicles_data is not None:

            for car in vehicles_data['vehicles']:

                fcw_assistant = self.fcw_factory.get_fcw(car['fcw_assistant'])

                self.vehicles.append(
                    vehicle.Vehicle(
                        weight=car['weight'],
                        has_abs=True if car['has_abs'] == 'true' else False,
                        max_speed=car['max_speed'],
                        area=car['area'],
                        fcw_assistant=fcw_assistant,
                        name=car['name']
                    )
                )

    def __prepare_vehicle_drivers(self):

        drivers_data = load_data('SimulationData/driver_data.json')

        if drivers_data is not None:

            for rider in drivers_data['drivers']:

                self.vehicle_drivers.append(vehicle_driver.VehicleDriver(
                    age=rider['age']
                ))

    def __prepare_road(self):

        road_data = load_data('SimulationData/road_data.json')

        if road_data is not None:

            for point in road_data['points']:
                self.road.append(_RoadPoint(
                    road_info=point.road_info,
                    position=point.position,
                    velocity=point.velocity,
                    acceleration=point.acceleration,
                    distance=point.distance,
                    steep=point.steep,
                    angle=point.angle
                ))

        else:
            raise Exception('Road data was not loaded')

    def start_simulation(self):

        car = self.vehicles[0]
        rider = self.vehicle_drivers[0]

        car.start_engine(driver_age=rider.age)

        for point in self.road:
            car.update_imu(
                velocity=point.velocity,
                road_info=point.road_info,
                distance=point.distance,
                acceleration=point.acceleration,
                steep=point.steep,
                angle=point.angle
            )

            car.move_vehicle(new_position=point.position)


fcw_simulation = FCWSimulation()

fcw_simulation.start_simulation()
