# Contains the Kinematic bicycle model and Dynamic bicycle model

import yaml
import numpy as np


# Kinematic bicycle model around the centre of gravity
class KinematicBicycleModel:
    """

	State [x, y, theta, delta]:

		x = vehicles relative x position
		y = vehicles relative y position
		theta = vehicles heading angle
		delta = vehicles steering angle

	Input [v, psi]:

		v = velocity of the vehicle
		psi = rate of steering angle change

	Coefficients:

		lF = 0 # dist from CoG ref to front axle
		lR = 0 # dist from CoG ref to rear axle

	GetNextState(v, psi):

		function that takes in the velocity and rate of steering change at a time and computes the next state for the vehicle

	"""

    # state variables
    x = 0
    y = 0
    theta = 0
    delta = 0
    slipAngle = 0

    # input variables
    v = 0
    psi = 0

    # time derivative
    dt = 1

    # state array
    State = np.array([x, y, theta, delta])

    # input array
    Input = np.array([v, psi])

    # vehicle characteristics:
    lF = 0
    lR = 0
    maxVelocity = 0  # maximum velocity of the car
    maxSteeringAngle = 0  # +- maximum steering angle of the car

    # initialises the class and vehicle coefficients
    def __init__(self):
        print("[INFO] Reading in vehicle characteristics and constraints from model.yaml")

        # reading in .yaml file
        with open(r'utils/model.yaml') as file:
            Yaml = yaml.load(file, Loader=yaml.FullLoader)

            self.lF = Yaml['lF']
            self.lR = Yaml['lR']
            self.maxVelocity = Yaml['maxV']
            self.maxSteeringAngle = Yaml['maxSA']

    # checks vehicle constraints
    def CheckConstraints(self):

        if self.delta > self.maxSteeringAngle:
            self.delta = self.maxSteeringAngle

        if self.delta < -self.maxSteeringAngle:
            self.delta = -self.maxSteeringAngle

        if self.v > self.maxVelocity:
            self.v = self.maxVelocity

    # gets the next state of the car
    def GetNextState(self, v, psi):

        # applying new velocity
        self.v = v

        # applying change of steering rate to steering angle (rads/sec)
        self.delta = self.delta + (self.dt * psi)

        # checking constraints
        self.CheckConstraints()

        # getting slip angle
        self.slipAngle = np.arctan(self.lR / (self.lR + self.lF) * np.tan(self.delta))

        # getting next
        self.x = self.x + self.dt * (self.v * np.cos(self.theta + self.slipAngle))
        self.y = self.y + self.dt * (self.v * np.sin(self.theta + self.slipAngle))
        self.theta = self.theta + self.dt * self.v / self.lF * np.sin(self.slipAngle)

        # updating State
        self.State = np.array([self.x, self.y, self.theta, self.delta])

        return self.State


# Dynamic bicycle model around the centre of gravity
class DynamicBicycleModel:
    """

	State [x, y, phi, vX, vY, omega] in the inertial frame

		x = vehicles x position
		y = vehicles y position
		phi = heading angle of the car
		vX = longitudinal velocity of the vehicle
		vY = lateral velocity of the vehicle
		r = yaw rate
		psi = steering angle of the vehicle
		t = driver command corresponding to a desired acceleration


	Input [d, psi]:

		_psi = rate of steering angle change
		_t = applied driver command

	Coefficients:

		m = mass of the vehicle
		lF = 0 # dist from CoG ref to front axle
		lR = 0 # dist from CoG ref to rear axle11
		cB, cC, cD = parameters describe the semi-emperical curve for the tires
		cD = drag coefficient
		cRR = rolling resistance coefficient
		cDTM = drive train motor coefficient

		etNextState(d, psi):

	function that takes in the velocity and rate of steering change at a time and computes the next state for the vehicle

	"""

    # state variables
    x = 0
    y = 0
    phi = 0
    vX = 0.01
    vY = 0.01
    r = 0
    delta = 0
    t = 0

    # input variables
    _delta = 0
    _t = 0

    # time derivative
    dt = 1

    # state array
    State = np.array([x, y, phi, vX, vY, r])

    # input array
    Input = np.array([_delta, _t])

    # vehicle characteristics:
    m = 0
    cB = 0
    cC = 0
    cD = 0
    cDr = 0
    lF = 0
    lR = 0
    iZ = 0
    cRR = 0
    cDTM = 0
    maxVelocity = 0  # maximum velocity of the car
    maxSteeringAngle = 0  # +- maximum steering angle of the car

    # initialises the class and its constants
    def __init__(self):
        print("[INFO] Reading in vehicle characteristics and constraints from model.yaml")

        # reading in .yaml file
        with open(r'utils/model.yaml') as file:
            Yaml = yaml.load(file, Loader=yaml.FullLoader)

            self.m = Yaml['m']
            self.cB = Yaml['cB']
            self.cC = Yaml['cC']
            self.cD = Yaml['cD']
            self.lF = Yaml['lF']
            self.lR = Yaml['lR']
            self.iZ = Yaml['iZ']
            self.cRR = Yaml['cRR']
            self.cDr = Yaml['cDr']
            self.cDTM = Yaml['cDTM']
            self.maxVelocity = Yaml['maxV']
            self.maxSteeringAngle = Yaml['maxSA']

    # checks vehicle constraints
    def CheckConstraints(self):

        if self.delta > self.maxSteeringAngle:
            self.delta = self.maxSteeringAngle

        if self.delta < -self.maxSteeringAngle:
            self.delta = -self.maxSteeringAngle

    # gets the next state of the car
    def GetNextState(self, _delta, _t):

        # applying change in steering angle
        self.delta = self.delta + self.dt * _delta

        # applying change in driver command
        self.t = self.t + self.dt * _t

        aR = np.arctan((self.vY - (self.lR * self.r)) / self.vX)
        aF = np.arctan((self.vY - (self.lF * self.r)) / self.vX) - self.delta

        # assuming that the coefficients for both wheels are the same
        FrY = self.cD * np.sin(self.cC * np.arctan(self.cB * aR))
        FfY = self.cD * np.sin(self.cC * np.arctan(self.cB * aF))

        Fx = self.cDTM * self.t - self.cRR - (self.cDr * self.vX ** 2)

        # checking constraints
        # self.CheckConstraints()

        self.x = self.x + self.dt * (self.vX * np.cos(self.phi) - self.vY * np.sin(self.phi))
        self.y = self.y + self.dt * (self.vX * np.sin(self.phi) + self.vY * np.cos(self.phi))
        self.phi = self.phi + self.dt * self.r

        self.vX = self.vX + self.dt * (1 / self.m * (Fx - (FfY * np.sin(self.delta)) + (self.m * self.vY * self.r)))
        self.vY = self.vY + self.dt * (1 / self.m * (FrY + (FfY * np.cos(self.delta)) - (self.m * self.vX * self.r)))
        self.r = self.r + self.dt * (1 / self.iZ * (FfY * self.lF * np.cos(self.delta) - FrY * self.lF))


if __name__ == "__main__":
    print("[INFO] This is to be used as a library not an executable")
