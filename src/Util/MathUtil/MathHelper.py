import cv2 as cv
import numpy as np
import Util.VisionUtil.VisionUtil as VisionUtil
from math import atan2, cos, sin, sqrt, pi

from scipy.spatial import distance as dist

# VisionUtil = VisionUtil.VisionUtil

class MathHelper:
    
    horizontal = [1., 0.]

    def getPrincipalAxes(contourPoints):
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv.PCACompute2(contourPoints, mean)
        cntr = (int(mean[0, 0]), int(mean[0, 1]))
        x = [eigenvectors[0][0], eigenvectors[1][0]]
        y = [eigenvectors[0][1], eigenvectors[1][1]]
        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        return x, y, cntr

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
    
    def getBoundingBoxPoints(points):
        x, y, w, h = cv.boundingRect(points)
        boundingBoxPoints = np.array([[x, y],
                                    [x + w, y],
                                    [x + w, y - h],
                                    [x, y - h]
                                    ], dtype=np.float32)
        # return the bounding box verticies, the area of the bounding box, and the aspect ratio of the width and height of the boudning box
        return boundingBoxPoints, w * h, w / h
    
    def getReferenceVector(pts):
        # first sort the points in a clockwise manner
        pts = MathHelper.sortRectPoints(pts)

        # construct our reference vector and normalize it
        referenceVector = [pts[1][0] - pts[0][0], pts[0][1] - pts[1][1]]
        referenceVector = MathHelper.norm(referenceVector)

        return referenceVector

    # get the midpoint of a contour
    def getMidpoint(pts):
        return 0

    def rotatePoint(pt, angle):
        x = (pt[0] * np.cos(angle)) - (pt[1] * np.sin(angle))
        y = (pt[1] * np.cos(angle)) + (pt[0] * np.sin(angle))
        return [x, y]

    def dot(a, b):     
        return (a[0] * b[0]) + (a[1] * b[1])

    def norm(vector):
        return vector / MathHelper.getLength(vector)

    def getLength(vector):
        return np.linalg.norm(vector)

    def getRelativeAngleDirection(a, b):
        return ((a[0] * b[1]) - (a[1] * b[0])) > 0

    def getAngle(a, b, signedRange = None):
        rotation = np.arccos(round(MathHelper.dot(a, b), 6) / round((MathHelper.getLength(a) * MathHelper.getLength(b)), 6))
        if signedRange is not None:
            sign = MathHelper.getRelativeAngleDirection(a, b)
            if (not sign):
                if (signedRange):
                    rotation = rotation * -1.0
                else :
                    rotation = (2 * np.pi) - rotation
        return rotation