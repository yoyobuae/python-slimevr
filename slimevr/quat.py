import math

class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def normalize(self):
        mag = self.magnitude()
        self.x /= mag
        self.y /= mag
        self.z /= mag

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

class Quaternion:
    def __init__(self, x=None, y=None, z=None, w=None):
        self.x = x or 0.0
        self.y = y or 0.0
        self.z = z or 0.0
        self.w = w or 1.0

    @staticmethod
    def fromAxisAngle(x, y, z, angle):
        axis = Vector(x, y, z)
        axis.normalize()
        s = math.sin(math.radians(angle)/2)
        return Quaternion(axis.x * s, axis.y * s, axis.z * s, math.cos(math.radians(angle)/2))
