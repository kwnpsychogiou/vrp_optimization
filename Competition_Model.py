import random
import math
import copy

class Node:
    # instance variables for class Node
    def __init__(self, id, tp, dem, xx, yy):
        self.id = id
        self.type = tp
        if tp == 1:
            self.loadingCost = 5/60
        elif tp == 2:
            self.loadingCost = 15/60
        elif tp == 3:
            self.loadingCost = 25/60
        elif tp == 0:
            self.loadingCost = 0
        self.demand = dem
        self.x = xx
        self.y = yy
        self.isTabuTillIterator = -1


class Model:

    # instance variables
    def __init__(self):
        self.allNodes = []
        self.customers = []
        self.time_matrix = []
        self.capacity = -1
        self.swapMoveTime_matrix = []

    def BuildModel(self):
        random.seed(1)
        depot = Node(0, 0, 0, 50, 50)
        self.allNodes.append(depot)
        self.capacity = 3000
        totalCustomers = 200 
        for i in range(0, totalCustomers): 
            tp = random.randint(1, 3)
            dem = random.randint(1, 5) * 100
            x = random.randint(0, 100)
            y = random.randint(0, 100)
            cust = Node(i + 1, tp, dem, x, y)
            self.allNodes.append(cust)
            self.customers.append(cust)
        self.time_matrix = [[0.0 for j in range(0, len(self.allNodes))] for k in range(0, len(self.allNodes))]
        for i in range(0, len(self.allNodes)):
            for j in range(0, len(self.allNodes)):
                if i != j:
                    source = self.allNodes[i]
                    target = self.allNodes[j]
                    dx_2 = (source.x - target.x)**2
                    dy_2 = (source.y - target.y) ** 2
                    dist = round(math.sqrt(dx_2 + dy_2))
                    
                    self.time_matrix[i][j] = dist / 35
        self.swapMoveTime_matrix = copy.deepcopy(self.time_matrix)
        for i in range(0, len(self.allNodes)):
            self.swapMoveTime_matrix[i][0] = 0.0

class Route:
    # instance variables for class Route
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfIds = []
        self.sequenceOfNodes.append(dp)
        self.sequenceOfIds.append(dp.id)
        self.cost = 0
        self.capacity = cap
        self.load = 0