SENSOR_OFFLINE = 0
SENSOR_OK = 1
SENSOR_ERROR = 2

class Sensor:
    def __init__(self, name, type_, id_, address, rotation):
        self.name = name
        self.type = type_
        self.id = id_
        self.address = address
        self.rotation = rotation

    def getSensorId(self):
        return self.id
    def getSensorType(self):
        return self.type
    def getSensorState(self):
        return SENSOR_OK
