# File: System.py
# Classes: Model(), Cost()

import numpy as np

def TaylorMethod(model, x0, u=None, dt=1e-3):
    return x0 + model(x0, u);

class Cost:
    pass;

class Model:
    def __init__(self, F, model_type='discrete', dt=1e-3):
        pass;