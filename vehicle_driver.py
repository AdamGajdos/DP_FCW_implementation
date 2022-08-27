class VehicleDriver:
    age: int

    def __init__(self, age):
        if age < 18:
            raise Exception('Legal age of driving is at least 18')
        else:
            self.age = age
