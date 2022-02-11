# This is a library that contains the various kinematic and dynamic models used within MPC.py 

import yaml
import numpy as np
# class that represents the 'kinematic bicycle model' for the 'centre of gravity' (CoG) reference point
class KinematicBicycleModel:

	""" 
	State [x, y, theta, delta]:

		x = vehicles relative x position
		y = vehicles relative y postition 
		theta = vehicles heading angle
		delta = vehicles steering angle

	Input [v, psi]:

		v = velocity of the vehicle
		psi = rate of steering change

	GetNextState(v, psi):

		function that takes in the velocity and rate of steering change at a time and computes the next state for the vehicle

	"""

	# state variables
	x = 0
	y = 0 
	theta = 0
	delta = 0
	slipAngle = 0 # 

	# time derivative
	dt = 1

	# input variables
	v = 0
	psi = 0

	# state array
	State = np.array([x, y, theta, delta])

	# input array
	Input = np.array([v, psi])

	# vehicle characteristics:
	lF = 0 # dist from CoG ref to front axle
	lR = 0 # dist from CoG ref to rear axle
	maxVelocity = 0 # maximum velocity of the car 
	maxSteeringAngle = 0 # +- maximum steering angle of the car

	# initialises the class and its constants
	def __init__(self):
		print("[INFO] Reading in vehicle characteristics and constraints from model.yaml")

		# reading in .yaml file
		with open(r'utils/model.yaml') as file:
			Yaml = yaml.load(file, Loader = yaml.FullLoader)

			self.lF = Yaml['lF']
			self.lR = Yaml['lR']
			self.maxVelocity = Yaml['maxV']
			self.maxSteeringAngle = Yaml['maxSA']

	# checks vehicle constraints
	def CheckConstraints(self, v, psi):
		
		if self.delta > self.maxSteeringAngle:
			self.delta = self.maxSteeringAngle

		if self.delta < -self.maxSteeringAngle:
			self.delta = -self.maxSteeringAngle

	# gets the next state of the car
	def GetNextState(self, v, psi):

		# applying change of steering rate to steering angle (degrees/sec)
		self.delta = self.delta + (self.dt * psi)

		# converting steering angle to radians (np.trig takes a rad)
		self.delta = np.deg2rad(self.delta)

		# getting slip angle
		self.slipAngle = np.arctan(self.lR / (self.lR + self.lF) * np.tan(self.delta))

    		# getting next state
		self.x = self.x + self.dt * (v * np.cos(self.theta + self.slipAngle) ) 
		self.y = self.y + self.dt * (v * np.sin(self.theta + self.slipAngle) ) 
		self.theta = self.theta + self.dt * v / self.lF * np.sin(self.slipAngle)

		# updating State array (returning theta in degrees)
		self.State = np.array([self.x, self.y, np.rad2deg(self.theta), self.delta])

		#print(self.State)

		return self.State


# class that represents the 'dynamic bicycle model' for the 'centre of gravity' (CoG) reference point
class DynamicBicycleModel:
	
	# initialises the class and its constants
	def __init__(self):
		print("[INFO] Reading in vehicle characteristics and constraints from model.yaml")

		# reading in .yaml file
		with open(r'utils/model.yaml') as file:
			Yaml = yaml.load(file, Loader = yaml.FullLoader)

			self.lF = Yaml['lF']
			self.lR = Yaml['lR']
			self.maxVelocity = Yaml['maxV']
			self.maxSteeringAngle = Yaml['maxSA']

