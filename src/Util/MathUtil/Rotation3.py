import numpy as np
import math

class Rotation3:
    def __init__(self, rotationMatrix):
        self.rotationMatrix = rotationMatrix
        self.eulerAngles = Rotation3.rotationMatrixToEulerAngles(rotationMatrix)

    def inverse(self):
        return Rotation3(np.transpose(self.rotationMatrix))

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
 
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(R) :
        assert(Rotation3.isRotationMatrix(R))
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])

    def __add__(self, other):
        sum = np.zeros((3, 3), dtype=np.float32)
        for i in range(3):
            for j in range(3):
                sum[i][j] = self.rotationMatrix[i][j] + other.rotationMatrix[i][j]
        return Rotation3(sum)

    def __str__(self):
        return 'Yaw:' + str(round(self.eulerAngles[0], 2)) + ', Pitch:' + str(round(self.eulerAngles[1], 2)) + ', Roll:' + str(round(self.eulerAngles[2], 2))