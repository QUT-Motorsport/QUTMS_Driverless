# An implementation of a model predictive controller for QUT Motorsport

# Importing some libraries
import time
import turtle
from utils.SystemModels import KinematicBicycleModel

# Model Predictive Controller class
class MPC:

	# Init
	def __init__(self):

		self.vehicle = KinematicBicycleModel()

		self.s = turtle.getscreen()
		self.t = turtle.Turtle()
		#self.t.penup()

		self.v = 0
		self.psi = 0

		#self.vehicle.GetNextState(2,-5)

		self.Predict()

	def f(self):
		self.v = self.v + 1
  
	def b(self):
   		self.v = self.v - 1
	  
	def l(self):
	    self.psi = self.psi - 1
	  
	def r(self):
	    self.psi = self.psi + 1

	def Predict(self):

		# To do: 
		# get next track point 
		# loop through different optimised options for input velocity and rate of steering change
		# return new state for car

		while(True):

			# getting key press
			self.s.listen()
			self.s.onkeypress(self.f, "Up")  # when up is pressed pen will go up
			self.s.onkeypress(self.b, "Down")  # when down is pressed pen will go down
			self.s.onkeypress(self.l, "Left")  # when left is pressed pen will go left
			self.s.onkeypress(self.r, "Right")  # when right is pressed pen will go right

			# getting state change
			self.vehicle.GetNextState(self.v,self.psi)

			# setting heading 
			self.t.setheading(self.vehicle.theta)
			self.t.goto(self.vehicle.x, self.vehicle.y)

			time.sleep(0.5)



# Main 
def main():
	mpc = MPC()

if __name__ == "__main__":
	main() 

