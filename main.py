import json
import Algorithms.fcw_factory
import fcw_assistant
import location
import vector_custom
import vehicle
import vehicle_driver
import vehicle_imu


class _RoadPoint:
    position: location.Location
    velocity: float
    road_info: vehicle_imu.RoadInfo
    distance: float
    acceleration: float
    deceleration: float
    steep: vehicle_imu.SteepSign
    angle: float
    delay: float

    def __init__(self, position, velocity, road_info, distance, acceleration, deceleration, steep, angle, delay):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.distance = distance
        self.road_info = road_info
        self.steep = steep
        self.angle = angle
        self.delay = delay


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

                fcw_assist = fcw_assistant.FCWAssistant(algorithm=self.fcw_factory.get_fcw(car['fcw_assistant']))

                self.vehicles.append(
                    vehicle.Vehicle(
                        weight=car['weight'],
                        has_abs=car['has_abs'],
                        max_speed=car['max_speed'],
                        area=car['area'],
                        fcw_assistant=fcw_assist,
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

        #road_data = load_data('SimulationData/road_data.json')
        #road_data = load_data('SimulationData/road_data_generated.json')
        road_data = load_data('SimulationData/road_data_generated_handgen.json')

        if road_data is not None:

            for point in road_data['points']:


                # position = vector_custom.Vector(x=point['position']['x'],
                #                                 y=point['position']['y'],
                #                                 z=point['position']['z'])

                position = location.Location(longitude=point['position']['longitude'],
                                             latitude=point['position']['latitude'])

                # road_info = vehicle_imu.RoadInfo(road_type=vehicle_imu.RoadType[point['road_info']['type']],
                #                                  condition=vehicle_imu.RoadCondition[point['road_info']['condition']])

                road_info = vehicle_imu.RoadInfo(road_type=vehicle_imu.RoadType[point['road_info']['road_type']],
                                                 condition=vehicle_imu.RoadCondition[point['road_info']['condition']])

                steep = vehicle_imu.SteepSign[point['steep']]

                self.road.append(_RoadPoint(
                    road_info=road_info,
                    position=position,
                    velocity=point['velocity'],
                    acceleration=point['acceleration'],
                    deceleration=point['deceleration'],
                    distance=point['distance'],
                    steep=steep,
                    angle=point['angle'],
                    delay=point['delay']
                ))

        else:
            raise Exception('Road data was not loaded')

    def start_simulation(self):

        rider = self.vehicle_drivers[0]

        for car in self.vehicles:

            car.start_engine(driver_age=rider.age)

            for point in self.road:
                car.update_imu(
                    velocity=point.velocity,
                    road_info=point.road_info,
                    distance=point.distance,
                    acceleration=point.acceleration,
                    deceleration=point.deceleration,
                    steep=point.steep,
                    angle=point.angle,
                    delay=point.delay
                )

                car.move_vehicle(new_position=point.position)

            print('\n')
