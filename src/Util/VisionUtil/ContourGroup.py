import cv2 as cv
import numpy as np
import Util.MathUtil.MathHelper as MathHelper
import Util.MathUtil.Line as Line
import Util.VisionUtil.VisionUtil as VisionUtil

MathHelper = MathHelper.MathHelper
Line = Line.Line
VisionUtil = VisionUtil.VisionUtil

class ContourGroup:
    def __init__(self, contours, frameCenter, useConvexHull, numOfCorners):
        # Save the contours that comprise the group
        self.contours = contours

        # Combine the points from the contours
        self.vertices = ContourGroup.combinePoints(contours)

        # Find the convex hull of the contour group
        self.useConvexHull = useConvexHull
        self.convexHull = cv.convexHull(self.vertices)

        # Apply k-means if there are duplicates
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        if useConvexHull:
            if (len(self.convexHull) > numOfCorners) and (numOfCorners is not 0):
                self.convexHull = cv.kmeans(self.convexHull.astype(dtype=np.float32), numOfCorners, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)[2]
                self.convexHull = self.convexHull.reshape((numOfCorners, 1, 2)).astype(int)
        else:
            if (len(self.vertices) > numOfCorners) and (numOfCorners is not 0):
                self.vertices = cv.kmeans(self.vertices.astype(dtype=np.float32), numOfCorners, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)[2]
                self.vertices = self.vertices.reshape((numOfCorners, 1, 2)).astype(int)

        # Find the area of the contour
        self.area = ContourGroup.combineArea(contours)

        # Obtain the straight bounding box
        self.boundingBoxPoints, self.boundingBoxArea, self.boundingBoxAspectRatio = VisionUtil.getBoundingBoxPoints(self.vertices)
        x, y, self.boundingBoxWidth, self.boundingBoxHeight = cv.boundingRect(self.vertices)
        self.boundingBoxUpperLeftPoint = (x, y)
        self.boundingBoxLowerRightPoint = (x + self.boundingBoxWidth, y + self.boundingBoxHeight)
        self.boundingBoxPoints = [self.boundingBoxUpperLeftPoint, [x + self.boundingBoxWidth, y], self.boundingBoxLowerRightPoint, [x, y + self.boundingBoxHeight]]

        # Find the rotated rect and it's area
        rect = cv.minAreaRect(self.vertices)
        _, (width, height), _ = rect
        if (width < height):
            self.tshort = width
            self.tlong = height
        else:
            self.tshort = height
            self.tlong = width
        box = np.int0(cv.boxPoints(rect))
        self.rotatedRect = [box]
        self.rotatedRectArea = cv.contourArea(box)
        
        # Get the center of the group
        self.midpoint = tuple(np.average(self.vertices, axis=0).ravel().astype(int))

        # Find the direction vector using the rotated rect and create a Line instance of it
        self.directionVector = VisionUtil.getReferenceVector(box)
        self.rotation = MathHelper.getAngle(MathHelper.horizontal, self.directionVector)
        self.referenceVector = Line(self.directionVector, self.midpoint)
        self.contourLine = Line(self.directionVector, [self.midpoint[0], frameCenter[1] * 2 - self.midpoint[1]])

        # Finally, sort the image points
        self.vertices = VisionUtil.sortImgPts(self.vertices, self.directionVector, self.midpoint)

        # Get the distance to the center of the frame
        self.distanceToCenter = np.linalg.norm(np.array([self.midpoint[0] - frameCenter[0], frameCenter[1] - self.midpoint[1]]))
    
    def combinePoints(contours):
        pts = np.concatenate((contours[0].vertices, contours[1].vertices), axis=0)
        i = 2
        while (i < len(contours)):
            pts = np.concatenate((pts, contours[i].vertices), axis=0)
            i = i + 1
        return pts

    def combineArea(contours):
        area = 0
        for contour in contours:
            area = area + contour.area
        return area