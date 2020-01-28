import Util.MathUtil.Vector3 as Vector3
import Util.MathUtil.Rotation3 as Rotation3

Vector3 = Vector3.Vector3
Rotation3 = Rotation3.Rotation3

class RigidTransform3:
    def __init__(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

    def inverse(self, other):
        return RigidTransform3(self.translation.negate(), self.rotation.inverse())
    
    def __add__(self, other):
        return RigidTransform3(self.translation + other.translation, self.rotation + other.rotation)

    def __str__(self):
        return str(self.translation) + '\n' + str(self.rotation)