import numpy as np

class Line:
	def __init__(self, directionVector, point):
		self.directionVector = directionVector
		self.point = point
		self.a = directionVector[0]
		self.b = directionVector[1]
		self.xSubNought = self.point[0]
		self.ySubNought = self.point[1]
		
	def getPoint(self, t, rounded):
		x = self.xSubNought + (self.a * t)
		y = self.ySubNought - (self.b * t)
		if rounded:
			x = int(round(x))
			y = int(round(y))
		return (x, y)
	
	# returns the point at which this line intersects another
	def intersects(self, other):
		a = np.array([
			[self.a, -1 * other.a],
			[self.b, -1 * other.b]
			], dtype=np.float32)
        
		c = np.array([
			[other.xSubNought + (-1 * self.xSubNought)],
			[other.ySubNought + (-1 * self.ySubNought)],
			], dtype=np.float32)
        
		intersects = True

		try:
			a_inv = np.linalg.inv(a)
		except:
			print('these two lines do not intersect!')
			intersects = False

		if intersects:
			result = np.matmul(a_inv, c)
			
			# now we calculate the point at which it intersects given t and s
			x = round(self.xSubNought + self.a * result[0][0])
			y = round(self.ySubNought + self.b * result[0][0])

			return [x, y]