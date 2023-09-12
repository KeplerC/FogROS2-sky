import argparse
import os
import yaml
import csv
import threading
import subprocess
import time
import numpy as np
from enum import Enum
from scipy.optimize import NonlinearConstraint, minimize, Bounds
import math
from itertools import product
from scipy.optimize import curve_fit
from models import Model, GPUModel

class OptimizationTest:

    def __init__(self):
        objective_function = self.timeFunction()
        print("NEWWW")
        bounds = Bounds(0,64)
        constraints = self.createConstraints()
        optimization_result = minimize(
            objective_function,
            x0=8,
            method="SLSQP",
            bounds=bounds,
            constraints=constraints,
            tol=1e-5,
            options={'maxiter': 10000}
        )
        print(optimization_result)
        print(self.costFunction()(61.19751838))

    def createConstraints(self):
        constraints = []
        # time_lower_bound = 0
        # time_upper_bound = 2700
        # time_constraint = NonlinearConstraint(
        #     self.timeFunction(), time_lower_bound, time_upper_bound
        # )
        # constraints.append(time_constraint)
        cost_lower_bound = 0
        cost_upper_bound = 1.5
        cost_constraint = NonlinearConstraint(self.costFunction(),cost_lower_bound,cost_upper_bound)
        constraints.append(cost_constraint)
        return constraints
    
    def costFunction(self):
        def costFn(x):
            seconds_per_step = 0.4148+(36.15634286 / x)
            total_time_in_seconds = (
                seconds_per_step * 1000
            ) + 191.1068
            dollars_per_second = 9.98058958e-5 +1.88507551e-5 * x
            cost = dollars_per_second * total_time_in_seconds
            return cost

        return costFn

    def timeFunction(self):
        def timeFn(x):
            seconds_per_step = 0.4148+(36.15634286 / x)
            total_time_in_seconds = (
                seconds_per_step * 1000
            ) + 191.1068
            return total_time_in_seconds

        return timeFn

print("Hello world")
optimization_test = OptimizationTest()
print("GOOD BYE WORLD")
