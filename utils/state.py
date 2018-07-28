import sys
import numpy as np
import math


INF = sys.float_info.max


class State(object):
    heuristic = ""
    connectionType = ""
    startState = None
    goalState = None
    stepSizeX = 0
    stepSizeY = 0
    stepSizeTheta = 0
    tolValue = 0
    G = float("inf")
    E = float("inf")
    env = None
    handler = []
    robot = None
    nodesDict = dict()
    stateList = []
    edgeCost = 1
    hMatrix = [[5,5,5,5],[INF,4,INF,4],[INF,3,INF,3],[INF,2,INF,3],[INF,1,INF,3],[0,1,2,3]]

    def __init__(self, config, pConfig, goalConfig=None):
        self._config = config
        self._parent = pConfig
        if goalConfig==None:
            self._hValue = euclideanMetric(self._config, self.goalState._config)
        else:
            self._hValue = euclideanMetric(self._config, goalConfig)
        self._gValue = sys.float_info.max
        self._fValue = 0
        self._edgeCosts = 1
        self._rhsValue = sys.float_info.max

    @property
    def gValue(self):
        return self._gValue

    @gValue.setter
    def gValue(self, value):
        self._gValue = value


    @property
    def hValue(self):
        return self._hValue

    @hValue.setter
    def hValue(self, value):
        self._hValue = value

    @property
    def fValue(self):
        return self._fValue


    @fValue.setter
    def fValue(self, value):
        self._fValue = value

    @property
    def rhsValue(self):
        return self._rhsValue

    @rhsValue.setter
    def rhsValue(self, value):
        self._rhsValue = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, conf):
        self._config = conf

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, conf):
        self._parent = conf

    def getCosts(self):
        return self._edgeCosts

def getEightConnectedNeighbors(state, stepSizeX, stepSizeY, stepSizeTheta):
    temp = []
    nextCells = []

    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            for k in [-1, 0, 1]:
                if i == j == k == 0:
                    continue
                if (state[2] + k * stepSizeTheta) >= math.pi:
                    Z = state[2] - 2 * math.pi
                elif (state[2] + k * stepSizeTheta) <= -1 * math.pi:
                    Z = state[2] + 2 * math.pi
                else:
                    Z = state[2]
                temp = [state[0] + i * stepSizeX, state[1] + j * stepSizeY, Z + k * stepSizeTheta]
                nextCells.append(temp)
    return nextCells

def euclideanMetric(current, goal):
    temp = [current[i]-goal[i] for i in range(len(current))]
    temp = np.asarray(temp)
    heuristic = np.linalg.norm(temp,2)
    return heuristic

def manhattanMetric(current, goal):
    temp = [current[i] - goal[i] for i in range(len(current))]
    temp = np.asarray(temp)
    heuristic = np.linalg.norm(temp,1)
    return heuristic