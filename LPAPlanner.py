import sys
import copy
import heapq
from state import *

def CalculateKey(u):
    return [min(u._gValue, u._rhsValue)+u._hValue, min(u._gValue, u._rhsValue)]

def initialize(startConf, goalConf, Env, rob, heuris, connection, stepX, stepY, stepTheta, tol):
    U = []
    State.startState = State(startConf, None, goalConf)
    State.goalState = State(goalConf, None, goalConf)
    State.connectionType = connection
    State.heuristic = heuris
    State.env = Env
    State.robot = rob
    State.stepSizeX = stepX
    State.stepSizeY = stepY
    State.stepSizeTheta = stepTheta
    State.tolValue = tol

    for indState in State.stateList:
        indState._rhsValue = sys.float_info.max
        indState._gValue = sys.float_info.max
    State.startState._rhsValue = 0
    # tempState = State(State.startState, None)
    heapq.heappush(U, (State.startState, [State.startState._hValue,0]))
    return U

def updateVertex(u, U):
    if u is not State.startState:
        minTemp = sys.float_info.max
        for predecessors in getSuccessors(u):
            temp = predecessors._gValue + State.edgeCost
            if temp<minTemp:
                minTemp = temp
        u._rhsValue = minTemp
    if u in U:
        U.remove(u)
    if u._gValue != u._rhsValue:
        heapq.heappush(U, (u, CalculateKey(u)))
        # U.append()

def drawState(handler, state, env, clr = np.array(((0, 0, 1)))):
    point = copy.deepcopy(state)
    point[2] = 0.1
    handler.append(env.plot3(points=point, pointsize=8.0, colors=clr))


def getSuccessors(indState):
    allStates = []
    possibleStates = []
    if State.connectionType == "eight":
        allStates = getEightConnectedNeighbors(indState._config, State.stepSizeX, State.stepSizeY, State.stepSizeTheta)
    for transition in allStates:
        State.robot.SetActiveDOFValues(transition)
        if not State.env.CheckCollision(State.robot):
            possibleStates.append(State(transition, indState._config))
        else:
            drawState(State.handler, transition, State.env, np.array(((1, 0, 0))))
    return possibleStates

def computeShortestPath(U):
    while (U[0][1]<CalculateKey(State.goalState) or State.goalState._rhsValue) != State.goalState._gValue:
        u = U.pop(0)[0]
        if u._gValue > u._rhsValue:
            u._gValue = u._rhsValue
            for s in getSuccessors(u):
                updateVertex(s, U)
        else:
            u._gValue = sys.float_info.max
            for s in getSuccessors(u)+[u]:
                updateVertex(s, U)

def LPAPlanner(startConfig, goalConfig, env, robot, heuristic, connectionType, stepSizeX, stepSizeY, stepSizeTheta, tolValue):
    openList = initialize(startConfig, goalConfig, env, robot, heuristic, connectionType, stepSizeX, stepSizeY, stepSizeTheta, tolValue)

    while True:
        computeShortestPath(openList)
        changes = input("Make changes in edge cost")
        for change in changes:
            updateVertex(change[v], openList)
