import Util.MathUtil.Rotation3 as Rotation3
import numpy as np

Rotation3 = Rotation3.Rotation3

class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

        self.length = np.linalg.norm([x, y, z])

    def scale(self, scale):
        return Vector3(self.x * scale, self.y * scale, self.z * scale)

    def negate(self):
        return Vector3(self.x * -1.0, self.y * -1.0, self.z * -1.0)

    def norm(self):
        return Vector3(self.x / self.length, self.y / self.length, self.z / self.length)

    def dot(self, other):
        return np.dot([self.x, self.y, self.z], [other.x, other.y, other.z])

    def cross(self, other):
        x, y, z = np.cross([self.x, self.y, self.z], [other.x, other.y, other.z])
        return Vector3(x, y, z)

    def rotate(self, rotation):
        result_x = x * rotation.rotationMatrix[0][0] + y * rotation.rotationMatrix[1][0] + z * rotation.rotationMatrix[2][0]
        result_y = x * rotation.rotationMatrix[0][1] + y * rotation.rotationMatrix[1][1] + z * rotation.rotationMatrix[2][1]
        result_z = x * rotation.rotationMatrix[0][2] + y * rotation.rotationMatrix[1][2] + z * rotation.rotationMatrix[2][2]
        return Vector3(result_x, result_y, result_z)

    def __mul__(self, other):
        return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
    
    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __str__(self):
        return 'X:' + str(self.x) + ', Y:' + str(self.y) + ', Z:' + str(self.z)