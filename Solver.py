from Competition_Model import *
from SolutionDrawerVRP import *
import copy
import random
import math
import time

class Solution:
    def __init__(self):
        self.cost = 0.0
        self.totalCost = 0.0 
        self.routes = []
        self.trucks = []

class RelocationMove(object):
    def __init__(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = None

    def Initialize(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = 10 ** 9


class SwapMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = 10 ** 9

class TwoOptMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = 10 ** 9

class Solver:
    def __init__(self, m, time_start):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.capacity = m.capacity
        self.time_matrix = m.time_matrix
        self.swapMoveTime_matrix = m.swapMoveTime_matrix
        self.sol = None
        self.bestSolution = None
        self.overallBestSol = None
        self.minTabuTenure = 10
        self.maxTabuTenure = 50
        self.tabuTenure = 25
        self.prioritizedMove = [-1, -1, -1, -1, 0.0]
        self.rcl_size = 6
        self.time_start = time_start

    def solve(self):
        
        self.createInitialSol()
        self.TabuSearch()
        # self.TestSolution()
        self.ReportSolution(self.sol.routes)

        return self.sol

    def createInitialSol(self):
        self.sol = Solution()
        self.create_trucks()
        self.create_routes()
        self.sorted_savings_dict = self.savings()
        while len(self.sol.routes) != 0:  
            self.addInTrucks()
        for i in self.sol.trucks:
            i.sequenceOfNodes.append(self.depot)
            i.sequenceOfIds.append(0)
        max = -1
        totCost = 0
        for t in self.sol.trucks:
            totCost += t.cost
            if t.load > 3000:
                print("You have exceeded the available capacity")
            if t.cost >= max:
                max = t.cost
        self.sol.routes = self.sol.trucks
        self.sol.cost = max
        self.sol.totalCost = totCost

    def TabuSearch(self):

        solution_cost_trajectory = []
        random.seed(108)
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top:TwoOptMove = TwoOptMove()

        SolDrawer.draw(0, self.sol.routes, self.allNodes)

        time_duration = 1200
        while time.time() < self.time_start + time_duration:
            operator = random.randint(0,2)

            rm.Initialize()
            sm.Initialize()
            top.Initialize()

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm, localSearchIterator)
                if rm.originRoutePosition is not None:
                    self.ApplyRelocationMove(rm, localSearchIterator)
            # Swaps
            elif operator == 1:
                self.FindBestSwapMove(sm, localSearchIterator)
                if sm.positionOfFirstRoute is not None:
                    self.ApplySwapMove(sm, localSearchIterator)
            elif operator == 2:
                self.FindBestTwoOptMove(top, localSearchIterator)
                if top.positionOfFirstRoute is not None:
                    self.ApplyTwoOptMove(top, localSearchIterator)

            # self.ReportSolution(self.sol)
            # self.TestSolution()
            solution_cost_trajectory.append(self.sol.cost)

            if (self.sol.cost < self.bestSolution.cost):
                self.bestSolution = self.cloneSolution(self.sol)

            localSearchIterator = localSearchIterator + 1

        SolDrawer.draw('final_ts', self.bestSolution.routes, self.allNodes)
        SolDrawer.drawTrajectory(solution_cost_trajectory)

        self.sol = self.bestSolution

    def LocalSearch(self):

        random.seed(101)
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top = TwoOptMove()

        while terminationCondition is False:
            operator = 2
            self.InitializeOperators(rm, sm, top)
            SolDrawer.draw(localSearchIterator, self.sol.routes, self.allNodes)

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm)
                if rm.originRoutePosition is not None:
                    if rm.moveCost < 0:
                        self.ApplyRelocationMove(rm)
                    else:
                        terminationCondition = True
            # Swaps
            elif operator == 1:

                self.FindBestSwapMove(sm)

                if sm.positionOfFirstRoute is not None:

                    if sm.moveCost < 0:
                        self.ApplySwapMove(sm)
                    elif sm.moveCost >= 0:
                        terminationCondition = True
            #TwoOpt
            elif operator == 2:
                flag = self.FindBestTwoOptMove(top, localSearchIterator)
                if top.positionOfFirstRoute is not None:
                    if top.moveCost < 0:
                        self.ApplyTwoOptMove(top, localSearchIterator)
                    else:
                        terminationCondition = True

            # self.TestSolution()

            if (self.sol.cost <= self.bestSolution.cost):
                self.bestSolution = self.cloneSolution(self.sol)

            localSearchIterator = localSearchIterator + 1

        self.sol = self.bestSolution
        return self.sol

    def VND(self):
        self.bestSolution = self.cloneSolution(self.sol)
        VNDIterator = 0
        kmax = 2
        rm = RelocationMove()
        sm = SwapMove()
        top = TwoOptMove()
        random.seed(101)
        draw = False
        k = random.randint(0, 2)

        while k <= kmax:
            self.InitializeOperators(rm, sm, top)
            if k == 1:
                self.FindBestRelocationMove(rm, VNDIterator)
                if rm.originRoutePosition is not None and rm.moveCost < 0:
                    self.ApplyRelocationMove(rm, VNDIterator)
                    if draw:
                        SolDrawer.draw(VNDIterator, self.sol.routes, self.allNodes)
                    VNDIterator = VNDIterator + 1
                    k = 0
                else:
                    k += 1
            elif k == 2:
                self.FindBestSwapMove(sm, VNDIterator)
                if sm.positionOfFirstRoute is not None and sm.moveCost < 0:
                    self.ApplySwapMove(sm, VNDIterator)
                    if draw:
                        SolDrawer.draw(VNDIterator, self.sol.routes, self.allNodes)
                    VNDIterator = VNDIterator + 1
                    k = 0
                else:
                    k += 1
            elif k == 0:
                flag = self.FindBestTwoOptMove(top, VNDIterator)
                if top.positionOfFirstRoute is not None and top.moveCost < 0:
                    self.ApplyTwoOptMove(top, VNDIterator)
                    if draw:
                        SolDrawer.draw(VNDIterator, self.sol.routes, self.allNodes)
                    VNDIterator = VNDIterator + 1
                    self.searchTrajectory.append(self.sol.cost)
                    k = 0
                else:
                    k += 1

            if (self.sol.cost <= self.bestSolution.cost):
                self.bestSolution = self.cloneSolution(self.sol)
        self.sol = self.bestSolution

    def cloneRoute(self, rt:Route):
        cloned = Route(self.depot, self.capacity)
        cloned.cost = rt.cost
        cloned.load = rt.load
        cloned.sequenceOfNodes = rt.sequenceOfNodes.copy()
        cloned.sequenceOfIds = rt.sequenceOfIds.copy()
        return cloned

    def cloneSolution(self, sol: Solution):
        cloned = Solution()
        for i in range (0, len(sol.routes)):
            rt = sol.routes[i]
            clonedRoute = self.cloneRoute(rt)
            cloned.routes.append(clonedRoute)
        cloned.cost = self.sol.cost
        return cloned

    def FindBestRelocationMove(self, rm, iterator):
        rt = self.FindMaxRoute(self.sol)
        rclRel = []
        for originRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[originRouteIndex]
            for targetRouteIndex in range (0, len(self.sol.routes)):
                rt2:Route = self.sol.routes[targetRouteIndex]
                for originNodeIndex in range (1, len(rt1.sequenceOfNodes) - 1):
                    for targetNodeIndex in range (0, len(rt2.sequenceOfNodes) - 1):

                        if originRouteIndex == targetRouteIndex and (targetNodeIndex == originNodeIndex or targetNodeIndex == originNodeIndex - 1):
                            continue

                        A = rt1.sequenceOfNodes[originNodeIndex - 1]
                        B = rt1.sequenceOfNodes[originNodeIndex]
                        C = rt1.sequenceOfNodes[originNodeIndex + 1]

                        F = rt2.sequenceOfNodes[targetNodeIndex]
                        G = rt2.sequenceOfNodes[targetNodeIndex + 1]

                        flag = False

                        if rt1 != rt2:
                            if rt2.load + B.demand > rt2.capacity:
                                continue

                        costAdded = self.swapMoveTime_matrix[A.id][C.id] + self.swapMoveTime_matrix[F.id][B.id] + self.swapMoveTime_matrix[B.id][G.id]
                        costRemoved = self.swapMoveTime_matrix[A.id][B.id] + self.swapMoveTime_matrix[B.id][C.id] + self.swapMoveTime_matrix[F.id][G.id]

                        originRtCostChange = self.swapMoveTime_matrix[A.id][C.id] - self.swapMoveTime_matrix[A.id][B.id] - self.swapMoveTime_matrix[B.id][C.id] - B.loadingCost
                        targetRtCostChange = self.swapMoveTime_matrix[F.id][B.id] + self.swapMoveTime_matrix[B.id][G.id] - self.swapMoveTime_matrix[F.id][G.id] + B.loadingCost

                        moveCost = costAdded - costRemoved

                        costRt1 = rt1.cost + originRtCostChange

                        if (self.MoveIsTabu(B, iterator, costRt1)):
                            continue

                        if rt1 == rt2:
                            costRt1 = rt1.cost + moveCost
                            if costRt1 > self.sol.cost:
                                flag2 = False
                        else:
                            costRt1 = rt1.cost + originRtCostChange
                        if rt1 == rt:
                            if costRt1 < self.sol.cost:
                                flag = True

                        flag2 = True
                        if (targetRtCostChange + rt2.cost > self.sol.cost):
                            flag2 = False

                        if flag and flag2:
                            rclRel = self.IdentifyBestRelocationMoves(rclRel, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, True)
                            continue
                            
                        if (moveCost < rm.moveCost) and abs(moveCost) > 0.0001 and flag2:
                            rclRel = self.IdentifyBestRelocationMoves(rclRel, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, False)

                            self.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, rm)
        if len(rclRel) > 0:
            tpl = rclRel[random.randint(0, len(rclRel) - 1)]
            self.StoreBestRelocationMove(tpl[0], tpl[1], tpl[2], tpl[3], tpl[4], tpl[5], tpl[6], rm)

    def IdentifyBestRelocationMoves(self, rcl, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, flag):
        if len(rcl) < self.rcl_size:
            new_tup = (originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, flag)
            rcl.append(new_tup)
            rcl.sort(key=lambda x: x[4])
        elif moveCost < rcl[-1][4] and rcl[-1][7] == False:
            rcl.pop(len(rcl) - 1)
            new_tup = (originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, flag)
            rcl.append(new_tup)
            rcl.sort(key=lambda x: x[4])
        return rcl

    def FindBestSwapMove(self, sm, iterator):
        rt = self.FindMaxRoute(self.sol)
        rclSwap = []
        for firstRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[firstRouteIndex]
            for secondRouteIndex in range (firstRouteIndex, len(self.sol.routes)):
                rt2:Route = self.sol.routes[secondRouteIndex]
                for firstNodeIndex in range (1, len(rt1.sequenceOfNodes) - 1):
                    startOfSecondNodeIndex = 1
                    if rt1 == rt2:
                        startOfSecondNodeIndex = firstNodeIndex + 1
                    for secondNodeIndex in range (startOfSecondNodeIndex, len(rt2.sequenceOfNodes) - 1):

                        a1 = rt1.sequenceOfNodes[firstNodeIndex - 1]
                        b1 = rt1.sequenceOfNodes[firstNodeIndex]
                        c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]

                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]

                        moveCost = None
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None
                        flag = False
                        flag2 = True
                        
                        if rt1 == rt2:
                            if firstNodeIndex == secondNodeIndex - 1:
                                costRemoved = self.swapMoveTime_matrix[a1.id][b1.id] + self.swapMoveTime_matrix[b1.id][b2.id] + self.swapMoveTime_matrix[b2.id][c2.id]
                                costAdded = self.swapMoveTime_matrix[a1.id][b2.id] + self.swapMoveTime_matrix[b2.id][b1.id] + self.swapMoveTime_matrix[b1.id][c2.id]
                                moveCost = costAdded - costRemoved
                            else:

                                costRemoved1 = self.swapMoveTime_matrix[a1.id][b1.id] + self.swapMoveTime_matrix[b1.id][c1.id]
                                costAdded1 = self.swapMoveTime_matrix[a1.id][b2.id] + self.swapMoveTime_matrix[b2.id][c1.id]
                                costRemoved2 = self.swapMoveTime_matrix[a2.id][b2.id] + self.swapMoveTime_matrix[b2.id][c2.id]
                                costAdded2 = self.swapMoveTime_matrix[a2.id][b1.id] + self.swapMoveTime_matrix[b1.id][c2.id]
                                moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)
                        else:
                            if rt1.load - b1.demand + b2.demand > self.capacity:
                                continue
                            if rt2.load - b2.demand + b1.demand > self.capacity:
                                continue

                            costRemoved1 = self.swapMoveTime_matrix[a1.id][b1.id] + self.swapMoveTime_matrix[b1.id][c1.id] + b1.loadingCost
                            costAdded1 = self.swapMoveTime_matrix[a1.id][b2.id] + self.swapMoveTime_matrix[b2.id][c1.id] + b2.loadingCost
                            costRemoved2 = self.swapMoveTime_matrix[a2.id][b2.id] + self.swapMoveTime_matrix[b2.id][c2.id] + b2.loadingCost
                            costAdded2 = self.swapMoveTime_matrix[a2.id][b1.id] + self.swapMoveTime_matrix[b1.id][c2.id] + b1.loadingCost

                            costChangeFirstRoute = costAdded1 - costRemoved1
                            costChangeSecondRoute = costAdded2 - costRemoved2

                            moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2)

                        if rt1 == rt2:
                            costChangeFirstRoute, costChangeSecondRoute = moveCost, moveCost
                        costRt1 = rt1.cost + costChangeFirstRoute
                        costRt2 = rt2.cost + costChangeSecondRoute
    
                        if self.MoveIsTabu(b1, iterator, costRt1) or self.MoveIsTabu(b2, iterator, costRt2):
                            continue

                        if costRt1 > self.sol.cost or costRt2 > self.sol.cost:
                                flag2 = False

                        if rt1 == rt:
                                if costRt1 < self.sol.cost and flag2: 
                                    flag = True
                        elif rt2 == rt:
                            if costRt2 < self.sol.cost and flag2: 
                                flag = True
                        
                        if flag and flag2:
                            rclSwap = self.IdentifyBestSwapMoves(rclSwap, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, True)
                        if (moveCost < sm.moveCost and flag2) or flag2 == True:
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)
                            rclSwap = self.IdentifyBestSwapMoves(rclSwap, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, False)
        
        if len(rclSwap) > 0:
            tpl = rclSwap[random.randint(0, len(rclSwap) - 1)]
            self.StoreBestSwapMove(tpl[0], tpl[1], tpl[2], tpl[3], tpl[4], tpl[5], tpl[6], sm)

    def IdentifyBestSwapMoves(self, rcl, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, flag):
    
         # Update rcl list
        if len(rcl) < self.rcl_size:
            new_tup = (firstRouteIndex,secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, flag)
            rcl.append(new_tup)
            rcl.sort(key=lambda x: x[4])
        elif moveCost < rcl[-1][4] and rcl[-1][7]==False:
            rcl.pop(len(rcl) - 1)
            new_tup = (firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, flag)
            rcl.append(new_tup)
            rcl.sort(key=lambda x: x[4])
        return rcl


    def FindBestTwoOptMove(self, top, iterator):
        
        rt = self.FindMaxRoute(self.sol)
        rcl = []
        
        for rtInd1 in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[rtInd1]
            for rtInd2 in range(rtInd1, len(self.sol.routes)):
                rt2:Route = self.sol.routes[rtInd2]
                for nodeInd1 in range(0, len(rt1.sequenceOfNodes) - 1):
                    start2 = 0
                    if (rt1 == rt2):
                        start2 = nodeInd1 + 2

                    for nodeInd2 in range(start2, len(rt2.sequenceOfNodes) - 1):
                        moveCost = 10 ** 9
                        costRt1 = 10 ** 9
                        costRt2 = 10 ** 9

                        A = rt1.sequenceOfNodes[nodeInd1]
                        B = rt1.sequenceOfNodes[nodeInd1 + 1]
                        K = rt2.sequenceOfNodes[nodeInd2]
                        L = rt2.sequenceOfNodes[nodeInd2 + 1]
                        flag2 = True
                        flag = False
                        if rt1 == rt2:
                            if nodeInd1 == 0 and nodeInd2 == len(rt1.sequenceOfNodes) - 2:
                                continue
                            costAdded = self.swapMoveTime_matrix[A.id][K.id] + self.swapMoveTime_matrix[B.id][L.id]
                            costRemoved = self.swapMoveTime_matrix[A.id][B.id] + self.swapMoveTime_matrix[K.id][L.id]
                            moveCost = costAdded - costRemoved
                            
                            if rt1 == rt and moveCost < 0:
                                flag = True

                            costRT = rt1.cost + moveCost
                            if self.MoveIsTabu(A, iterator, costRT) or self.MoveIsTabu(K, iterator, costRT):
                                continue
                        else:
                            if nodeInd1 == 0 and nodeInd2 == 0:
                                continue
                            if nodeInd1 == len(rt1.sequenceOfNodes) - 2 and  nodeInd2 == len(rt2.sequenceOfNodes) - 2:
                                continue
                            
                            if self.CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
                                continue
                            costAdded = self.swapMoveTime_matrix[A.id][L.id] + self.swapMoveTime_matrix[B.id][K.id]
                            costRemoved = self.swapMoveTime_matrix[A.id][B.id] + self.swapMoveTime_matrix[K.id][L.id]
                            moveCost = costAdded - costRemoved

                            rtb0Cost = self.CalculateSegCost(rt1.sequenceOfNodes, nodeInd1 + 1) # kostos uposinolou [B,..,0]
                            rtl0Cost = self.CalculateSegCost(rt2.sequenceOfNodes, nodeInd2 + 1) # kostos uposinolou [L,..,0]
                            costRt1 = rt1.cost - rtb0Cost - self.swapMoveTime_matrix[A.id][B.id] - B.loadingCost + rtl0Cost + self.swapMoveTime_matrix[A.id][L.id] + L.loadingCost
                            costRt2 = rt2.cost - rtl0Cost - self.swapMoveTime_matrix[K.id][L.id] - L.loadingCost + rtb0Cost + self.swapMoveTime_matrix[K.id][B.id] + B.loadingCost
                            
                            if costRt1 > self.sol.cost or costRt2 > self.sol.cost:
                                flag2 = False
                            
                            if rt1 == rt:
                                if costRt1 < self.sol.cost and flag2:
                                    flag = True
                            elif rt2 == rt:
                                if costRt2 < self.sol.cost and flag2:
                                    flag = True

                        
                            if self.MoveIsTabu(A, iterator, costRt1) or self.MoveIsTabu(K, iterator, costRt2):
                                continue
                        
                        if flag :
                            rcl = self.IdentifyBestTwoOptMoves(rcl, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, True)
                            continue
                        if (moveCost < top.moveCost and abs(moveCost) > 0.0001 and flag2):
                            top.moveCost = moveCost
                            rcl = self.IdentifyBestTwoOptMoves(rcl, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, False)
                            self.StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)
        if len(rcl) > 0:
            tpl = rcl[random.randint(0, len(rcl) - 1)]
            self.StoreBestTwoOptMove(tpl[0], tpl[1], tpl[2], tpl[3], tpl[4], top)
        return rcl

    def IdentifyBestTwoOptMoves(self, rcl, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, flag):
        if len(rcl) < self.rcl_size:
            new_tup = (rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, flag)
            rcl.append(new_tup)
            rcl.sort(key=lambda x: x[4])
        elif moveCost < rcl[-1][4] and rcl[-1][5] == False :
            rcl.pop()
            new_tup = (rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, flag)
            rcl.append(new_tup)
            rcl.sort(key=lambda x: x[4])
        return rcl


    def MoveIsTabu(self, n: Node, iterator, costRT):
        if costRT < self.sol.cost - 0.001:
            return False
        if iterator < n.isTabuTillIterator:
            return True
        return False

    def SetTabuIterator(self, n: Node, iterator):
        n.isTabuTillIterator = iterator + random.randint(self.minTabuTenure, self.maxTabuTenure)
    
    def CalculateSegCost(self, sequence, start):
        cost = 0
        for i in range(start, len(sequence) - 2):
            cost += self.swapMoveTime_matrix[sequence[i].id][sequence[i+1].id] + sequence[i + 1].loadingCost 
        return cost
    
    def ApplyRelocationMove(self, rm: RelocationMove, iterator):
        oldMaxCost = self.CalculateMaxCost(self.sol)

        originRt = self.sol.routes[rm.originRoutePosition]
        targetRt = self.sol.routes[rm.targetRoutePosition]

        B = originRt.sequenceOfNodes[rm.originNodePosition]

        if originRt == targetRt:
            del originRt.sequenceOfNodes[rm.originNodePosition]
            del originRt.sequenceOfIds[rm.originNodePosition]
            if (rm.originNodePosition < rm.targetNodePosition):
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition, B)
                targetRt.sequenceOfIds.insert(rm.targetNodePosition, B.id)
            else:
                targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
                targetRt.sequenceOfIds.insert(rm.targetNodePosition + 1, B.id)

            originRt.cost += rm.moveCost
        else:
            del originRt.sequenceOfNodes[rm.originNodePosition]
            del originRt.sequenceOfIds[rm.originNodePosition]
            targetRt.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)
            targetRt.sequenceOfIds.insert(rm.targetNodePosition + 1, B.id)
            originRt.cost += rm.costChangeOriginRt
            targetRt.cost += rm.costChangeTargetRt
            originRt.load -= B.demand
            targetRt.load += B.demand

        newMaxCost = self.CalculateMaxCost(self.sol)
        self.sol.cost = newMaxCost

        self.SetTabuIterator(B, iterator)


    def ApplySwapMove(self, sm, iterator):
        rt1Cost=0
        rt2Cost=0
        oldMaxCost = self.CalculateMaxCost(self.sol)
        rt1 = self.sol.routes[sm.positionOfFirstRoute]
        rt2 = self.sol.routes[sm.positionOfSecondRoute]

        b1 = rt1.sequenceOfNodes[sm.positionOfFirstNode]
        b2 = rt2.sequenceOfNodes[sm.positionOfSecondNode]
        rt1.sequenceOfNodes[sm.positionOfFirstNode] = b2
        rt2.sequenceOfNodes[sm.positionOfSecondNode] = b1
        rt1.sequenceOfIds[sm.positionOfFirstNode] = b2.id 
        rt2.sequenceOfIds[sm.positionOfSecondNode] = b1.id

        prin1=rt1.cost
        prin2=rt2.cost
        if (rt1 == rt2):
            rt1.cost += sm.moveCost
        else:
            rt1.cost = rt1.cost + sm.costChangeFirstRt 
            rt2.cost = rt2.cost + sm.costChangeSecondRt
            rt1.load = rt1.load - b1.demand + b2.demand
            rt2.load = rt2.load + b1.demand - b2.demand
            
        for n in range (0 , len(rt1.sequenceOfNodes) - 2):
                A = rt1.sequenceOfNodes[n]
                B = rt1.sequenceOfNodes[n + 1]
                rt1Cost += self.swapMoveTime_matrix[A.id][B.id] + B.loadingCost
        for n in range (0 , len(rt2.sequenceOfNodes) - 2):
                A = rt2.sequenceOfNodes[n]
                B = rt2.sequenceOfNodes[n + 1]
                rt2Cost += self.swapMoveTime_matrix[A.id][B.id] + B.loadingCost
        newMaxCost = self.CalculateMaxCost(self.sol)
        self.sol.cost = newMaxCost

        self.SetTabuIterator(b1, iterator)
        self.SetTabuIterator(b2, iterator)

    def ApplyTwoOptMove(self, top, iterator):
        rt1:Route = self.sol.routes[top.positionOfFirstRoute]
        rt2:Route = self.sol.routes[top.positionOfSecondRoute]
        if rt1 == rt2:
            # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
            reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
            reversedSegment2 = reversed(rt1.sequenceOfIds[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
            #lst = list(reversedSegment)
            #lst2 = list(reversedSegment)
            rt1.sequenceOfNodes[top.positionOfFirstNode + 1 : top.positionOfSecondNode + 1] = reversedSegment
            rt1.sequenceOfIds[top.positionOfFirstNode + 1 : top.positionOfSecondNode + 1] = reversedSegment2
            #reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
            #rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList

            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfFirstNode], iterator)
            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfSecondNode], iterator)

            rt1.cost += top.moveCost
            
        else:
            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]
            relocatedSegment2OfRt1 = rt1.sequenceOfIds[top.positionOfFirstNode + 1 :]
            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]
            relocatedSegment2OfRt2 = rt2.sequenceOfIds[top.positionOfSecondNode + 1 :]
            del rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]
            del rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]
            del rt1.sequenceOfIds[top.positionOfFirstNode + 1 :]
            del rt2.sequenceOfIds[top.positionOfSecondNode + 1 :]

            rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
            rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)
            rt1.sequenceOfIds.extend(relocatedSegment2OfRt2)
            rt2.sequenceOfIds.extend(relocatedSegment2OfRt1)

            self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfFirstNode], iterator)
            self.SetTabuIterator(rt2.sequenceOfNodes[top.positionOfSecondNode], iterator)

            self.UpdateRouteCostAndLoad(rt1)
            self.UpdateRouteCostAndLoad(rt2)
        newMaxCost = self.CalculateMaxCost(self.sol)
        self.sol.cost = newMaxCost

    def UpdateRouteCostAndLoad(self, rt: Route):
        tc = 0
        tl = 0
        for i in range(0, len(rt.sequenceOfNodes) - 2):
            A = rt.sequenceOfNodes[i]
            B = rt.sequenceOfNodes[i+1]
            tc += self.swapMoveTime_matrix[A.id][B.id] + B.loadingCost
            tl += B.demand
        rt.load = tl
        rt.cost = tc

    def ReportSolution(self, routes):
        f = open("sol.txt", "w")
        f.write(str(self.sol.cost))
        f.write("\n")
        for i in range(0, len(routes)):
            rt = routes[i]
            for j in range (0, len(rt.sequenceOfIds)-1):
                l = str(rt.sequenceOfIds[j])
                f.write(l)
                if j != len(rt.sequenceOfIds)-2:
                    f.write(",")
            f.write("\n")

    def StoreBestRelocationMove(self, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, originRtCostChange, targetRtCostChange, rm:RelocationMove):
        rm.originRoutePosition = originRouteIndex
        rm.originNodePosition = originNodeIndex
        rm.targetRoutePosition = targetRouteIndex
        rm.targetNodePosition = targetNodeIndex
        rm.costChangeOriginRt = originRtCostChange
        rm.costChangeTargetRt = targetRtCostChange
        rm.moveCost = moveCost

    def StoreBestSwapMove(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, sm):
        sm.positionOfFirstRoute = firstRouteIndex
        sm.positionOfSecondRoute = secondRouteIndex
        sm.positionOfFirstNode = firstNodeIndex
        sm.positionOfSecondNode = secondNodeIndex
        sm.costChangeFirstRt = costChangeFirstRoute
        sm.costChangeSecondRt = costChangeSecondRoute
        sm.moveCost = moveCost

    def StoreBestTwoOptMove(self, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
        top.positionOfFirstRoute = rtInd1
        top.positionOfSecondRoute = rtInd2
        top.positionOfFirstNode = nodeInd1
        top.positionOfSecondNode = nodeInd2
        top.moveCost = moveCost

    def CalculateMaxCost(self, sol):
        max = -1.0
        for i in range (0, len(sol.routes)):
            rt = sol.routes[i]
            if rt.cost > max:
                max = rt.cost
        return max
    
    def FindMaxRoute(self, sol):
        max = -1
        rtMax = sol.routes[0]
        for i in range (0, len(sol.routes)):
            rt = sol.routes[i]
            if rt.cost > max:
                max = rt.cost
                rtMax = rt
        return rtMax

    def InitializeOperators(self,rm, sm, top):
        rm.Initialize()
        sm.Initialize()
        top.Initialize()

    def TestSolution(self):
        for r in range (0, len(self.sol.routes)):
            rt: Route = self.sol.routes[r]
            rtCost = 0
            rtLoad = 0
            for n in range (0 , len(rt.sequenceOfNodes) - 2):
                A = rt.sequenceOfNodes[n]
                B = rt.sequenceOfNodes[n + 1]
                rtCost += self.time_matrix[A.id][B.id] + B.loadingCost
                rtLoad += B.demand
            if abs(rtCost - rt.cost) > 0.0001:
                print ('Route Cost problem', abs(rtCost - rt.cost))
            if rtLoad != rt.load:
                print ('Route Load problem')

            maxcost=self.CalculateMaxCost(self.sol)
        if abs(maxcost- self.sol.cost) > 0.0001:
            print('Solution Cost problem')

    def CapacityIsViolated(self, rt1, nodeInd1, rt2, nodeInd2):
    
        rt1FirstSegmentLoad = 0
        for i in range(0, nodeInd1 + 1):
            n = rt1.sequenceOfNodes[i]
            rt1FirstSegmentLoad += n.demand
        rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad

        rt2FirstSegmentLoad = 0
        for i in range(0, nodeInd2 + 1):
            n = rt2.sequenceOfNodes[i]
            rt2FirstSegmentLoad += n.demand
        rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

        if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity):
            return True
        if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity):
            return True

        return False

    def findNode(self, sequence, id):
        for node in sequence:
            if node.id == id:
                return node

    def create_trucks(self):         
        minDistance = {}
        for j in range(1, len(self.allNodes)):
            minDistance[(0, j)] = self.time_matrix[0][j]
        minDistanceSorted = sorted(minDistance.items(), key=lambda x: x[1])

        minDistanceSorted_dict = {}
        for x in minDistanceSorted:
            minDistanceSorted_dict[(x[0][0], x[0][1])] = x[1]

        counter = 0 
        for x in minDistanceSorted_dict.keys():
            r = Route(self.allNodes[0], self.capacity)
            r.sequenceOfNodes.append(self.allNodes[x[1]])
            r.sequenceOfIds.append(self.allNodes[x[1]].id)
            r.load = r.load + self.allNodes[x[1]].demand
            r.cost += self.time_matrix[0][self.allNodes[x[1]].id] + self.allNodes[x[1]].loadingCost
            self.sol.trucks.append(r)
            counter += 1
            if counter==25:
                break

    def create_routes(self):
        for i in range(0,len(self.customers)) :
            find = False
            for t in self.sol.trucks:
                if t.sequenceOfIds[-1] == self.customers[i].id:
                    find = True
            if find == False:
                r = Route(self.allNodes[0], self.capacity)
                r.sequenceOfNodes.append(self.customers[i])
                r.sequenceOfIds.append(self.customers[i].id)
                r.load = r.load + self.customers[i].demand
                r.cost += self.time_matrix[0][self.customers[i].id] + self.allNodes[self.customers[i].id].loadingCost 
                self.sol.routes.append(r)

    def savings(self):
        savings_dict = {}
        for i in range(1, len(self.allNodes)):
            if i+1 < len(self.allNodes):
                for j in range(i+1, len(self.allNodes)):
                        saving = self.time_matrix[i][0] + self.time_matrix[0][j] - self.time_matrix[i][j]
                        savings_dict[(i, j)] = saving
        sorted_keys = sorted(savings_dict.items(), key=lambda x: x[1], reverse=True) # sort savings in descending order
        sorted_savings_dict = {}
        for x in sorted_keys:
            sorted_savings_dict[(x[0][0], x[0][1])] = x[1]
        return sorted_savings_dict

    def addInTrucks(self):
        for x in self.sorted_savings_dict.keys():
            flag = False
            route1 = None
            route2 = None
            for t in self.sol.trucks:

                if len(t.sequenceOfIds) == 9:
                    continue
                if (x[0] in t.sequenceOfIds and x[1] in t.sequenceOfIds):
                    flag = True
                    break

   
                elif x[0] in t.sequenceOfIds and t.sequenceOfIds[-1] == x[0] :
          
                    for r in self.sol.routes:
                        if x[1] in r.sequenceOfIds and r.sequenceOfIds[1] == x[1]:
                            route1 = t
                            route2 = r
      
                elif x[1] in t.sequenceOfIds and t.sequenceOfIds[-1] == x[1] :
                  
                    for r in self.sol.routes:
                        if x[0] in r.sequenceOfIds and r.sequenceOfIds[1] == x[0]:
                            route1 = t
                            route2 = r
                   
            if flag is False and route1 is not None and route2 is not None:
               
                if (route1.load + route2.load) <= route1.capacity:
                    new_route = Route(self.allNodes[0], self.capacity)
              
                    for i in route1.sequenceOfIds:
                        if i!= 0 :
                            current_node:Node = self.findNode(route1.sequenceOfNodes, i)
                            new_route.cost += self.time_matrix[new_route.sequenceOfIds[-1]][i] + current_node.loadingCost
                            new_route.sequenceOfIds.append(i)
                            new_route.sequenceOfNodes.append(current_node)
                            new_route.load += current_node.demand
                            
                    for j in route2.sequenceOfIds:
                        if j!= 0:
                            current_node:Node = self.findNode(route2.sequenceOfNodes, j)
                            new_route.cost += self.time_matrix[new_route.sequenceOfIds[-1]][j] + current_node.loadingCost
                            new_route.sequenceOfIds.append(j)
                            new_route.sequenceOfNodes.append(current_node)
                            new_route.load += current_node.demand
                        

                    self.sol.trucks.append(new_route)
                    self.sol.trucks.remove(route1)
                    self.sol.routes.remove(route2)
        
                    del self.sorted_savings_dict[x]
                    return


               

