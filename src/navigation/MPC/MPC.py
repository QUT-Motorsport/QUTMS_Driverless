# An implementation of a model predictive controller for QUT Motorsport

# Importing some libraries
import time
import turtle
from utils.SystemModels import *


# Model Predictive Controller class
class MPC:

    # Init
    def __init__(self):
        raise NotImplementedError()

    # Predict
    def predict(self):
        raise NotImplementedError()


# Main
def main():
    DyB = DynamicBicycleModel()
    DyB.GetNextState(10, 1)


if __name__ == "__main__":
    main()
