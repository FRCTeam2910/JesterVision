from scipy.spatial import distance as dist
import cv2 as cv
import numpy as np
import collections
import Util.MathUtil.MathHelper as MathHelper
import Util.MathUtil.Vector3 as Vector3
import Util.MathUtil.Rotation3 as Rotation3
import Util.MathUtil.RigidTransform3 as RigidTransform3

MathHelper = MathHelper.MathHelper
Vector3 = Vector3.Vector3
Rotation3 = Rotation3.Rotation3
RigidTransform3 = RigidTransform3.RigidTransform3

class VisionUtil:

    axis = np.float32([[0,0,0], [5,0,0], [0,5,0], [0,0,5]]).reshape(-1,3)

    def getMidPoint(pts):
        moment = cv.moments(pts)
        midPtX = int(moment['m10'] / moment['m00'])
        midPtY = int(moment['m01'] / moment['m00'])
        return (midPtX, midPtY)

    def sortImgPts(imgpts, x, midpt):
        numOfPoints = len(imgpts)
        pts = {}
        for i in range(numOfPoints):
            vector = MathHelper.norm([imgpts[i][0][0] - midpt[0], midpt[1] - imgpts[i][0][1]])
            angle = MathHelper.getAngle(x, vector, False)
            pts[angle] = imgpts[i]
        
        pts = collections.OrderedDict(sorted(pts.items()))
        
        sortedPts = np.zeros((numOfPoints, 1, 2), dtype=np.int32)
        j = 0
        for i in pts:
            sortedPts[j] = pts[i]
            j+=1

        return sortedPts

    def sortRectPoints(pts):
        # sort the points based on their x-coordinates
        xSorted = pts[np.argsort(pts[:, 0]), :]
    
        # grab the left-most and right-most points from the sorted
        # x-roodinate points
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
    
        # now, sort the left-most coordinates according to their
        # y-coordinates so we can grab the top-left and bottom-left
        # points, respectively
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost
    
        # now that we have the top-left coordinate, use it as an
        # anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean
        # theorem, the point with the largest distance will be
        # our bottom-right point
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        (br, tr) = rightMost[np.argsort(D)[::-1], :]
    
        # return the coordinates in top-left, top-right,
        # bottom-right, and bottom-left order
        return np.array([tl, tr, br, bl], dtype="float32")

    def getTranslation(cameraMatrix, distortionCoefficients, objectPoints, imagePoints, range=None):
        # Sort out our range first
        if range is not None:
            objectPoints = objectPoints[range[0] - 1:range[1]]
            imagePoints = imagePoints[range[0] - 1:range[1]]
        
        # Perform pose estimation, obtain the rotation matrix
        retval, rotationVector, translationVector = cv.solvePnP(objectPoints, imagePoints, cameraMatrix, distortionCoefficients)
        rotationMatrix, jacobianMatrix = cv.Rodrigues(rotationVector)

        # Put the results into a RigidTransform3
        translation = Vector3(translationVector[0], translationVector[1], translationVector[2])
        rotation = Rotation3(rotationMatrix)
        rigidTransform = RigidTransform3(translation, rotation)

        # Project the 3D points onto the image plane
        imgpts, jac = cv.projectPoints(VisionUtil.axis, rotationVector, translationVector, cameraMatrix, distortionCoefficients)

        return rigidTransform, imgpts
    
    def getBoundingBoxPoints(points):
        x, y, w, h = cv.boundingRect(points)
        boundingBoxPoints = np.array([[x, y],
                                    [x + w, y],
                                    [x + w, y - h],
                                    [x, y - h]
                                    ], dtype=np.float32)
        # return the bounding box verticies, the area of the bounding box, and the aspect ratio of the width and height of the boudning box
        return np.array(boundingBoxPoints, dtype=np.uint8), w * h, w / h
    
    def getReferenceVector(points):
        points = VisionUtil.sortRectPoints(points)

        lowestPointVal = points[0][1]
        lowestPointIndex = 0
        for i in range(len(points)):
            if (points[i][1] > lowestPointVal):
                lowestPointVal = points[i][1]
                lowestPointIndex = i
            
        points = np.concatenate((points[lowestPointIndex:], points[:lowestPointIndex]))

        # vector a
        a = [points[1][0] - points[0][0], points[0][1] - points[1][1]]
        width = MathHelper.getLength(a)

        # vector b
        b = [points[2][0] - points[1][0], points[1][1] - points[2][1]]
        height = MathHelper.getLength(b)

        # vector d
        d = [points[3][0] - points[0][0], points[0][1] - points[3][1]]
        d = MathHelper.norm(d)

        angle = np.degrees(MathHelper.getAngle(MathHelper.horizontal, d))

        if (width > height):
            angle = 270 + angle

        return [np.cos(np.radians(angle)), np.sin(np.radians(angle))]

    def getAngleToTarget(rvecs):
        angle = np.pi - np.arccos(MathHelper.dot(MathHelper.norm([rvecs[2][0], rvecs[2][2]]), [0, 1]))
        crossProduct = np.cross([0, 0, 1], rvecs[2])
        if (crossProduct[1] < 0):
            angle*=-1
        return angle