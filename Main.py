#from TSP_Model import Model
from Solver import *
import time

time_start = time.time()
m = Model()
m.BuildModel()
s = Solver(m, time_start)
sol = s.solve()
