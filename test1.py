from __future__ import print_function
import numpy as np
from sklearn.cluster import AgglomerativeClustering
from sklearn.cluster import SpectralClustering
from sklearn.cluster import SpectralBiclustering
from sklearn.cluster import SpectralCoclustering
from sklearn.cluster import KMeans
from sklearn.cluster import Birch
from sklearn.cluster import MiniBatchKMeans
from ortools.algorithms import pywrapknapsack_solver
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from collections import OrderedDict
import matplotlib.pyplot as plt
from matplotlib import collections
from datetime import datetime
import KMedoids
import operator
import random
import elkai
import math
import sys


def generateRandomSwap(clusters, nodes, m, currentSolution, tabuTag, iteration, alfa, beta, forceSwap):
    fromPossibleClusters = []
    toPossibleClusters = []
    toOtherPossibleClusters = []

    for i in range(0, m):
        # nodesDividedByClusters[i] = list(clusters[i].keys())

        # print("LUNGHEZZA: " + str(len(clusters[i])))
        # print("LUNGHEZZA MINIMA: " + str(alfa * ((nodes - 2) / m)))
        # print("LUNGHEZZA MASSIMA: " + str(beta * ((nodes - 2) / m)))

        if len(clusters[i]) > alfa * ((nodes - 2) / m):
            fromPossibleClusters.append(i)

        if len(clusters[i]) < beta * ((nodes - 2) / m):
            toPossibleClusters.append(i)

    fromTotalItems = []
    if forceSwap:
        for c in range(0, len(currentSolution)):
            if c in fromPossibleClusters:
                for j in range(1, len(currentSolution[c]) - 1):
                    if iteration > tabuTag[currentSolution[c][j]]:
                        fromTotalItems.append((currentSolution[c][j], c))

        if len(fromTotalItems) == 0:
            for c in range(0, len(currentSolution)):
                if c in fromPossibleClusters:
                    for j in range(1, len(currentSolution[c]) - 1):
                        fromTotalItems.append((currentSolution[c][j], c))
    else:
        for c in fromPossibleClusters:
            for item in clusters[c].keys():
                if iteration > tabuTag[item]:
                    fromTotalItems.append((item, c))

        if len(fromTotalItems) == 0:
            for c in fromPossibleClusters:
                for item in clusters[c].keys():
                    fromTotalItems.append((item, c))

    swappedNodeIndex = random.randint(0, len(fromTotalItems) - 1)
    swappedNode = fromTotalItems[swappedNodeIndex]
    fromCluster = swappedNode[1]

    for i in range(0, m):
        if swappedNode[1] != i:
            if i in toPossibleClusters:
                toOtherPossibleClusters.append(i)

    toClusterIndex = random.randint(0, len(toOtherPossibleClusters) - 1)
    toCluster = toOtherPossibleClusters[toClusterIndex]

    return swappedNode[0], fromCluster, toCluster


def calculateForceSwapIterations(nodes):
    return round(1 / 50 * (nodes ** 1.3) + 4)


def calculateRestartIterations(nodes):
    return round(1 / 30 * (nodes ** 1.3) + 10)


def calculateBlockNodeIterations(nodes):
    return round(1.2 * math.log2(nodes))


def calculateMaxIterationsWithoutImprovement(nodes):
    return round(100 * ((500 / nodes) * math.exp(-(nodes / 1000))))


def destroyTours(currentSolution, totalCosts, tourProfits, packingCosts, service_cost, profits,
                 travellingTimesMatrix, solutionValue):
    # print("DESTROY TOUR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    # print("---")
    # print(currentSolution)
    nodesDeleted = []

    for i in range(0, m):
        # fraction = (1 - (tourProfits[i] / solutionValue)) / 2
        # print((int(len(currentSolution[i][1:-1]) / 3) + (len(currentSolution[i][1:-1]) % 3 > 0)))

        for k in range(0, (int(len(currentSolution[i][1:-1]) / 3) + (len(currentSolution[i][1:-1]) % 3 > 0))):
            tmp = math.inf

            for j in range(1, len(currentSolution[i]) - 1):
                node = currentSolution[i][j]
                previousNode = currentSolution[i][j - 1]
                nextNode = currentSolution[i][j + 1]
                ratio = profits[node] / (
                        service_cost[node] + travellingTimesMatrix[previousNode][node] + travellingTimesMatrix[node][
                    nextNode])

                if tmp > ratio:
                    tmp = ratio
                    nodeToDelete = node
                    prevNode = previousNode
                    nNode = nextNode
                    indexNode = j

            nodesDeleted.append((nodeToDelete, i))
            totalCosts[i] = totalCosts[i] + travellingTimesMatrix[prevNode][nNode] - (
                    service_cost[nodeToDelete] + travellingTimesMatrix[prevNode][nodeToDelete] +
                    travellingTimesMatrix[nodeToDelete][nNode])

            tourProfits[i] -= profits[nodeToDelete]
            currentSolution[i].pop(indexNode)
            packingCosts[i] -= service_cost[nodeToDelete]

    # print(currentSolution)
    # print("+++")
    return nodesDeleted


def modifyTour(tmax, tour, totalCost, profit, packingCost, travellingTimesMatrix, service_cost, profits, tabuTagOut,
               item, clusterIndex):
    tmp = math.inf
    tourModified = False

    for i in range(1, len(tour) - 1):
        if iteration > tabuTagOut[clusterIndex][tour[i]]:
            newTour = tour.copy()
            removedItem = newTour.pop(i)
            newProfit = profit - profits[removedItem] + profits[item]

            newTotalCost = totalCost - service_cost[removedItem] + service_cost[item]
            newTotalCost -= (travellingTimesMatrix[tour[i - 1]][removedItem] + travellingTimesMatrix[removedItem][
                tour[i + 1]])
            newTotalCost += (travellingTimesMatrix[tour[i - 1]][item] + travellingTimesMatrix[item][tour[i + 1]])

            if newTotalCost <= tmax:
                tourModified = True
                if tmp > newTotalCost:
                    deletedItem = removedItem
                    tmp = newTotalCost
                    modifiedTour = newTour
                    modifiedTour.insert(i, item)
                    profitTour = newProfit
                    newPackingCost = packingCost - service_cost[removedItem] + service_cost[item]
                    gap = profitTour - profit

    if tourModified:
        return modifiedTour, profitTour, deletedItem, tmp, newPackingCost, gap
    else:
        return None


def increaseTour(tmax, tour, totalCost, profit, packingCost, travellingTimesMatrix, service_cost, profits, item):
    tmp = math.inf
    tourIncreased = False

    for i in range(0, len(tour) - 1):
        newTotalCost = totalCost
        newProfit = profit + profits[item]

        newTotalCost -= travellingTimesMatrix[tour[i]][tour[i + 1]]
        newTotalCost += (travellingTimesMatrix[tour[i]][item] + travellingTimesMatrix[item][tour[i + 1]])
        newTotalCost += service_cost[item]

        if newTotalCost <= tmax:
            tourIncreased = True
            if tmp > newTotalCost:
                tmp = newTotalCost
                newTour = tour.copy()
                newTour.insert(i + 1, item)
                profitTour = newProfit
                newPackingCost = packingCost + service_cost[item]
                gap = profitTour - profit

    if tourIncreased:
        return newTour, profitTour, tmp, newPackingCost, gap
    else:
        return None


# def intraClusterFirstImprovment(newCurrentSolution, newCurrentTourProfit, inNodes, sortedOutNodes, Tmax,
#                                 packingWeight, start, end, travellingTimesMatrix, service_cost, profits):
#     for inNode in inNodes:
#         newSolution = newCurrentSolution.copy()
#         newSolution.remove(inNode)
#
#         for outNode in sortedOutNodes:
#             # profit, l = calculateTour(Tmax, packingWeight - service_cost[inNode] + service_cost[outNode],
#             #                           newSolution[1:-1] + [outNode], start, end, travellingTimesMatrix, service_cost,
#             #                           profits)
#
#             if newCurrentTourProfit < profit:
#                 newCurrentTourProfit = profit
#                 newCurrentSolution = l
#                 sortedOutNodes.remove(outNode)
#                 sortedOutNodes.append(inNode)
#                 sortedInNodes.append(outNode)
#                 sortedInNodes.remove(inNode)
#                 return sortedInNodes, sortedOutNodes, newCurrentTourProfit, newCurrentSolution
#
#     return ()


def intraClusterOptimization(cluster, tour, profits, service_cost, currentPackedItems,
                             travellingTimesMatrix, currentTourProfit, totalCost, alfa, beta, Tmax):
    bestSolution = tour
    bestSolutionValue = currentTourProfit
    bestTotalCost = totalCost

    BLOCK_INSERTING_NODE_ITERATIONS = 4
    BLOCK_EXTRACT_NODE_ITERATIONS = 2

    MAX_BLOCK_INSERTING_NODE_ITERATIONS = alfa
    MAX_BLOCK_EXTRACT_NODE_ITERATIONS = beta

    iteration = 0
    tabuTagIn = {}
    tabuTagOut = {}

    for item in cluster.keys():
        tabuTagOut[item] = -1
        tabuTagIn[item] = -1

    # newSolutionValue = []
    #
    # newSolutionValue.append(currentTourProfits.copy())

    # totalProfits = 0
    # for i in range(0, m):
    #     for item in clusters[i]:
    #         if item not in currentSolution[i]:
    #             profit, l = calculateTour(Tmax, currentWeight + service_cost[item], currentSolution[i][1:-1] + [item],
    #                                       start, end, travellingTimesMatrix, service_cost, profits)
    #
    #             if profit > currentTourProfits[i]:
    #                 print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    #                 newSolution[i] = l
    #                 newSolutionValue[i] = profit
    #
    #     totalProfits += currentTourProfits[i]

    # for i in range(0, m):
    #     profitWeightRatiosIn = {}
    #     profitWeightRatiosOut = {}
    #
    #     for j in range(1, len(currentSolution[i]) - 1):
    #         profitWeightRatiosIn[currentSolution[i][j]] = \
    #             profits[currentSolution[i][j]] / (service_cost[currentSolution[i][j]] + (travellingTimesMatrix[currentSolution[i][j - 1]][currentSolution[i][j]] + travellingTimesMatrix[currentSolution[i][j]][currentSolution[i][j + 1]]) / 2)
    #
    #     for item in clusters[i].keys():
    #         if item not in currentSolution[i]:
    #             profitWeightRatiosOut[item] = profits[item] / service_cost[item]
    #
    #     sorted_d = dict(sorted(profitWeightRatiosOut.items(), key=operator.itemgetter(1), reverse=True))
    #     sortedOutNodes[i] = list(sorted_d.keys())
    #     sorted_d = dict(sorted(profitWeightRatiosIn.items(), key=operator.itemgetter(1)))
    #     sortedInNodes[i] = list(sorted_d.keys())

    newCurrentSolution = tour.copy()

    while iteration < 100:
        currentProfit = -math.inf
        feasibleSolutionFound = False
        swap = None
        outNodes = []

        for item in cluster.keys():
            if item not in newCurrentSolution[1:-1]:
                if tabuTagIn[item] < iteration:
                    outNodes.append(item)

        for i in range(1, len(newCurrentSolution) - 1):
            if tabuTagOut[newCurrentSolution[i]] < iteration:
                PartialCost = totalCost - (
                        travellingTimesMatrix[newCurrentSolution[i - 1]][newCurrentSolution[i]]
                        + travellingTimesMatrix[newCurrentSolution[i]][newCurrentSolution[i + 1]])

                PartialCost = PartialCost - service_cost[newCurrentSolution[i]]

                for outNode in outNodes:
                    newTotalCost = PartialCost + service_cost[outNode] + \
                                   (travellingTimesMatrix[newCurrentSolution[i - 1]][outNode] +
                                    travellingTimesMatrix[outNode][
                                        newCurrentSolution[i + 1]])

                    if newTotalCost <= Tmax:
                        feasibleSolutionFound = True
                        profit = currentTourProfit - profits[newCurrentSolution[i]] + profits[outNode]

                        if currentProfit < profit:
                            currentProfit = profit
                            currentCost = newTotalCost
                            swap = (outNode, i)

        if feasibleSolutionFound:
            outNode = swap[0]
            position = swap[1]
            tabuTagIn[newCurrentSolution[position]] = iteration + BLOCK_INSERTING_NODE_ITERATIONS
            tabuTagOut[outNode] = iteration + BLOCK_EXTRACT_NODE_ITERATIONS
            newCurrentSolution[position] = outNode
            currentTourProfit = currentProfit
            totalCost = currentCost

        if bestSolutionValue < currentProfit:
            BLOCK_INSERTING_NODE_ITERATIONS /= 2
            BLOCK_EXTRACT_NODE_ITERATIONS /= 2
            bestSolutionValue = currentProfit
            bestTotalCost = currentCost
            bestSolution = newCurrentSolution.copy()
        else:
            if BLOCK_INSERTING_NODE_ITERATIONS < MAX_BLOCK_INSERTING_NODE_ITERATIONS:
                BLOCK_INSERTING_NODE_ITERATIONS += 1

            if BLOCK_EXTRACT_NODE_ITERATIONS < MAX_BLOCK_EXTRACT_NODE_ITERATIONS:
                BLOCK_EXTRACT_NODE_ITERATIONS += 1

        iteration += 1

    packingProfit = 0
    packingItems = list(set().union(currentPackedItems, bestSolution[1:-1]))
    for i in range(0, len(packingItems)):
        packingProfit += profits[packingItems[i]]

    return bestSolution, bestSolutionValue, bestTotalCost, packingItems, packingProfit

    # for i in range(1, len(tour) - 1):
    #     newSolution = newCurrentSolution.copy()
    #
    #     newTotalCost = totalCost - (travellingTimesMatrix[tour[i - 1]][tour[i]] + travellingTimesMatrix[tour[i]][tour[i + 1]])
    #     newTotalCost = newTotalCost - service_cost[tour[i]]
    #
    #     for outNode in outNodes:
    #         newTotalCost = newTotalCost + service_cost[outNode] + \
    #                        (travellingTimesMatrix[tour[i - 1]][outNode] + travellingTimesMatrix[outNode][tour[i + 1]])
    #
    #         if newTotalCost <= Tmax:
    #             profit = currentTourProfit - profits[tour[i]] + profits[outNode]
    #
    #             if newCurrentTourProfit < profit:
    #                 newCurrentTourProfit = profit
    #                 newSolution[i] = outNode
    #                 outNodes.remove(outNode)
    #                 outNodes.append(tour[i])
    #                 print("dsadasd " + str(newCurrentTourProfit))
    #                 print("AAAAAAAAAAAAA " + str(newTotalCost))
    #                 print(newCurrentSolution)
    #                 improvement = True

    # while improvement:
    #     newTour = tour.copy()
    #     retValue = intraClusterFirstImprovment(newTour, newCurrentTourProfit, inNodes,
    #                                            outNodes, Tmax, packingWeight, start, end, travellingTimesMatrix,
    #                                            service_cost, profits)
    #
    #     if retValue != ():
    #         exit()
    #         inNodes = retValue[0]
    #         outNodes = retValue[1]
    #         newCurrentTourProfit = retValue[2]
    #         newTour = retValue[3]
    #         clusterIndex = retValue[4]
    #     else:
    #         improvement = False


def restartConfiguration(clusters, coordinates, m, alfa):
    nodesToSwap = []

    for c in range(0, m):
        fromTotalItems = []
        nodesInCluster = list(clusters[c].keys())

        for i in range(0, int(len(clusters[c]) * alfa)):
            rnd = random.randint(0, len(nodesInCluster) - 1)
            fromTotalItems.append(nodesInCluster.pop(rnd))

        for item in fromTotalItems:
            minDistance = math.inf

            for k in range(0, m):
                if k != c:
                    centroidForCluster = np.zeros(2)
                    for j in clusters[k]:
                        centroidForCluster += coordinates[j]
                    centroidForCluster /= len(clusters[k])
                    tmp = np.linalg.norm(coordinates[item] - centroidForCluster)

                    if tmp < minDistance:
                        minDistance = tmp
                        toCluster = k

            nodesToSwap.append((item, c, toCluster))

    for (node, fromC, toC) in nodesToSwap:
        clusters[toC][node] = clusters[fromC][node]
        del clusters[fromC][node]


def calculateTourLKH(Tmax, total_weight, packed_items, start, end, travellingTimesMatrix, service_cost, profits):
    s = Tmax - total_weight
    profitWeightRatios = {}

    for item in packed_items:
        profitWeightRatios[item] = profits[item] / service_cost[item]

    sorted_d = dict(sorted(profitWeightRatios.items(), key=operator.itemgetter(1)))
    sortedNodes = list(sorted_d.keys())

    if len(sortedNodes) > 0:
        distanceMatrix = np.empty((len(sortedNodes) + 1, len(sortedNodes) + 1), dtype=np.uint8)

        for i in range(0, len(sortedNodes) + 1):
            for j in range(0, len(sortedNodes) + 1):
                distanceMatrix[i][j] = travellingTimesMatrix[([start] + sortedNodes)[i]][([start] + sortedNodes)[j]]

        totalCost, l = getFeasibleSolutionLKH(distanceMatrix, [start] + sortedNodes, travellingTimesMatrix, end, Tmax,
                                              s)

        while totalCost > Tmax:
            s = s + service_cost[sortedNodes.pop(0)]
            distanceMatrix = np.delete(distanceMatrix, 1, 0)
            distanceMatrix = np.delete(distanceMatrix, 1, 1)
            totalCost, l = getFeasibleSolutionLKH(distanceMatrix, [start] + sortedNodes, travellingTimesMatrix, end,
                                                  Tmax,
                                                  s)

        profit = 0
        for i in range(1, len(l) - 1):
            profit = profit + profits[l[i]]

        return profit, l, totalCost

    return 0, [], 0

def calculateTour(Tmax, total_weight, packed_items, start, end, travellingTimesMatrix, service_cost, profits):
    s = Tmax - total_weight
    profitWeightRatios = {}

    for item in packed_items:
        profitWeightRatios[item] = profits[item] / service_cost[item]

    sorted_d = dict(sorted(profitWeightRatios.items(), key=operator.itemgetter(1)))
    sortedNodes = list(sorted_d.keys())

    distanceMatrix = np.empty((len(sortedNodes) + 1, len(sortedNodes) + 1), dtype=np.uint8)

    for i in range(0, len(sortedNodes) + 1):
        for j in range(0, len(sortedNodes) + 1):
            distanceMatrix[i][j] = travellingTimesMatrix[([start] + sortedNodes)[i]][([start] + sortedNodes)[j]]

    totalCost, l = getFeasibleSolution(distanceMatrix, [start] + sortedNodes, travellingTimesMatrix, start, end, Tmax,
                                       s)

    while totalCost > Tmax:
        s = s + service_cost[sortedNodes.pop(0)]
        distanceMatrix = np.delete(distanceMatrix, 1, 0)
        distanceMatrix = np.delete(distanceMatrix, 1, 1)
        totalCost, l = getFeasibleSolution(distanceMatrix, [start] + sortedNodes, travellingTimesMatrix, start, end,
                                           Tmax, s)

    profit = 0
    for i in range(1, len(l) - 1):
        profit = profit + profits[l[i]]

    return profit, l, totalCost


def calculatePacking(cluster, service_cost, profits, Tmax):
    weights = []
    profitsForCluster = []

    for i in range(0, len(cluster)):
        weights.append(service_cost[cluster[i]])
        profitsForCluster.append(profits[cluster[i]])

    solver = pywrapknapsack_solver.KnapsackSolver(
        pywrapknapsack_solver.KnapsackSolver.
            KNAPSACK_DYNAMIC_PROGRAMMING_SOLVER, 'DynamicProgrammingKnapsack')

    solver.Init(profitsForCluster, [weights], [Tmax])
    computed_value = solver.Solve()

    packed_items = []
    total_weight = 0

    for i in range(len(cluster)):
        if solver.BestSolutionContains(i):
            packed_items.append(cluster[i])
            total_weight += service_cost[cluster[i]]

    return computed_value, packed_items, total_weight


def plotClusters(clusters, nodes, coordinates, currentSolution):
    labels_ = generateLabels(clusters, nodes)

    labelsColor = list(labels_.values())
    labelsData = list(labels_.keys())

    coords = np.array(list(coordinates.values()))
    plt.figure(figsize=(10, 7))
    plt.subplots_adjust(bottom=0.1)
    plt.scatter(coords[:, 0], coords[:, 1], c=np.array(labelsColor))

    for solution in currentSolution:
        x = []
        y = []
        for item in solution:
            x.append(coordinates[item][0])
            y.append(coordinates[item][1])

        plt.plot(x, y, linewidth=1)

    for label, x, y in zip(labelsData, coords[:, 0], coords[:, 1]):
        plt.annotate(
            label,
            xy=(x, y), xytext=(-3, 3),
            textcoords='offset points', ha='right', va='bottom')
    plt.show()


def generateLabels(clusters, nodes):
    labels = {}
    labels[0] = 0

    for i in range(0, len(clusters)):
        for item in clusters[i]:
            labels[item] = i + 1

    labels[nodes - 1] = 0
    sortedLabels = OrderedDict(sorted(labels.items()))

    return sortedLabels


def calculateSimplifiedSilhouettes(clusters, coordinates):
    aItem = 0
    bItem = math.inf
    silhouettes = {}

    for cluster in clusters:
        centroidForCluster = np.zeros(2)
        for i in cluster.keys():
            centroidForCluster += coordinates[i]
        centroidForCluster /= len(cluster)

        for item in cluster.keys():
            aItem = aItem + np.linalg.norm(coordinates[item] - centroidForCluster)

            for cluster1 in clusters:
                if cluster1 != cluster:
                    centroidForCluster1 = np.zeros(2)
                    for j in cluster1.keys():
                        centroidForCluster1 += coordinates[j]
                    centroidForCluster1 /= len(cluster1)
                    tmp = np.linalg.norm(coordinates[item] - centroidForCluster1)

                    if tmp < bItem:
                        bItem = tmp

            silhouettes[item] = (bItem - aItem) / max(aItem, bItem)

    return silhouettes


def packingHeuristic(clusters, currentBestPackingValues, service_cost, profits, nodes, m,
                     tabuTag, iteration, alfa, beta, currentSolution, forceSwap):
    bestPackingProfitFound = -math.inf
    # minDistance = math.inf
    # fromCluster = None
    # toCluster = None
    # nodesDividedByClusters = {}
    fromPossibleClusters = []
    toPossibleClusters = []

    # silhouettes = calculateSimplifiedSilhouettes(clusters, coordinates)

    for i in range(0, m):
        # nodesDividedByClusters[i] = list(clusters[i].keys())

        # print("LUNGHEZZA: " + str(len(clusters[i])))
        # print("LUNGHEZZA MINIMA: " + str(alfa * ((nodes - 2) / m)))
        # print("LUNGHEZZA MASSIMA: " + str(beta * ((nodes - 2) / m)))

        if len(clusters[i]) > alfa * ((nodes - 2) / m):
            # if similarSwap[i] > -3:
            fromPossibleClusters.append(i)

        if len(clusters[i]) < beta * ((nodes - 2) / m):
            # if similarSwap[i] < 3:
            toPossibleClusters.append(i)

    # print(fromPossibleClusters)
    # print(toPossibleClusters)

    fromTotalItems = []
    if forceSwap:
        for c in range(0, m):
            if c in fromPossibleClusters:
                for j in range(1, len(currentSolution[c]) - 1):
                    if iteration > tabuTag[currentSolution[c][j]]:
                        fromTotalItems.append((currentSolution[c][j], c))

        if len(fromTotalItems) == 0:
            for c in range(0, m):
                if c in fromPossibleClusters:
                    for j in range(1, len(currentSolution[c]) - 1):
                        fromTotalItems.append((currentSolution[c][j], c))
    else:
        for c in fromPossibleClusters:
            for item in clusters[c].keys():
                if iteration > tabuTag[item]:
                    fromTotalItems.append((item, c))

        if len(fromTotalItems) == 0:
            for c in fromPossibleClusters:
                for item in clusters[c].keys():
                    fromTotalItems.append((item, c))

    i = 0
    while i < len(fromTotalItems):
        item = fromTotalItems[i][0]
        c = fromTotalItems[i][1]
        clusteringDict = {}

        for k in toPossibleClusters:
            if k != c:
                clusteringDict[c] = clusters[c].copy()
                clusteringDict[k] = clusters[k].copy()

                clusteringDict[k][item] = clusteringDict[c][item]
                del clusteringDict[c][item]

                packingProfit = 0
                packing1 = calculatePacking(list(clusteringDict[c].keys()), service_cost, profits, tmax)
                packing2 = calculatePacking(list(clusteringDict[k].keys()), service_cost, profits, tmax)
                packingProfit += packing1[0]
                packingProfit += packing2[0]

                currentBestPackingProfit = 0
                currentBestPackingProfit += currentBestPackingValues[c]
                currentBestPackingProfit += currentBestPackingValues[k]

                if bestPackingProfitFound < packingProfit:
                    bestPackingProfitFound = packingProfit
                    packingsDict = {}
                    packingsDict[c] = packing1
                    packingsDict[k] = packing2
                    partition = clusteringDict.copy()
                    bestPacking = packingsDict
                    swappedNode = item

        i += 1

    return partition, bestPacking, swappedNode


def fastPackingHeuristic(clusters, packingValues, packingItems, packingCosts, service_cost, profits, nodes, m,
                         tabuTag, iteration, alfa, beta, currentSolution, forceSwap, Tmax):
    feasibleSwapFound = False
    bestSwapFound = -math.inf
    fromPossibleClusters = []
    toPossibleClusters = []

    # silhouettes = calculateSimplifiedSilhouettes(clusters, coordinates)

    for i in range(0, m):
        # nodesDividedByClusters[i] = list(clusters[i].keys())

        # print("LUNGHEZZA: " + str(len(clusters[i])))
        # print("LUNGHEZZA MINIMA: " + str(alfa * ((nodes - 2) / m)))
        # print("LUNGHEZZA MASSIMA: " + str(beta * ((nodes - 2) / m)))

        if len(clusters[i]) > alfa * ((nodes - 2) / m):
            # if similarSwap[i] > -3:
            fromPossibleClusters.append(i)

        if len(clusters[i]) < beta * ((nodes - 2) / m):
            # if similarSwap[i] < 3:
            toPossibleClusters.append(i)

    # print(fromPossibleClusters)
    # print(toPossibleClusters)

    fromTotalItems = []
    if forceSwap:
        for c in range(0, len(currentSolution)):
            if c in fromPossibleClusters:
                for j in range(1, len(currentSolution[c]) - 1):
                    if iteration > tabuTag[currentSolution[c][j]]:
                        fromTotalItems.append((currentSolution[c][j], c))

        if len(fromTotalItems) == 0:
            for c in range(0, len(currentSolution)):
                if c in fromPossibleClusters:
                    for j in range(1, len(currentSolution[c]) - 1):
                        fromTotalItems.append((currentSolution[c][j], c))
    else:
        for c in fromPossibleClusters:
            for item in clusters[c].keys():
                if iteration > tabuTag[item]:
                    fromTotalItems.append((item, c))

        if len(fromTotalItems) == 0:
            for c in fromPossibleClusters:
                for item in clusters[c].keys():
                    fromTotalItems.append((item, c))

    i = 0
    while i < len(fromTotalItems):
        item = fromTotalItems[i][0]
        c = fromTotalItems[i][1]

        feasibleSwapC = False
        bestPackingValueC = -math.inf
        bestPackingItemsC = packingItems[c]
        bestPackingCostC = packingCosts[c]

        if item in packingItems[c]:
            newPackingItems = packingItems[c].copy()
            newPackingItems.remove(item)
            packingValueC = packingValues[c] - profits[item]
            packingCostC = packingCosts[c] - service_cost[item]

            for el in clusters[c]:
                if el not in packingItems[c]:
                    packingValueC += profits[el]
                    packingCostC += service_cost[el]

                    if packingCostC <= Tmax:
                        feasibleSwapC = True
                        if packingCostC > bestPackingValueC:
                            bestPackingValueC = packingValueC
                            bestPackingCostC = packingCostC
                            bestPackingItemsC = newPackingItems + [el]

            if not feasibleSwapC:
                bestPackingValueC = packingValueC
                bestPackingCostC = packingCostC
                bestPackingItemsC = newPackingItems
        else:
            bestPackingValueC = packingValues[c]

        for k in toPossibleClusters:
            if k != c:
                clusteringDict = {c: clusters[c].copy(), k: clusters[k].copy()}
                clusteringDict[k][item] = clusteringDict[c][item]
                del clusteringDict[c][item]

                feasibleSwapK = False
                bestPackingValueK = -math.inf
                bestPackingItemsK = packingItems[k]
                bestPackingCostK = packingCosts[k]

                for node in packingItems[k]:
                    packingValueK = packingValues[k] - profits[node] + profits[item]
                    packingCostK = packingCosts[k] - service_cost[node] + service_cost[item]

                    if packingCostK <= Tmax:
                        feasibleSwapK = True
                        if packingValueK > bestPackingValueK:
                            bestPackingValueK = packingValueK
                            bestPackingCostK = packingCostK
                            l = packingItems[k].copy()
                            l.remove(node)
                            bestPackingItemsK = l + [item]

                if feasibleSwapK:
                    feasibleSwapFound = True
                    if bestSwapFound < bestPackingValueK:
                        bestPacking = {}
                        partition = clusteringDict.copy()

                        bestSwapFound = bestPackingValueK
                        bestPacking[c] = (bestPackingValueC, bestPackingItemsC, bestPackingCostC)
                        bestPacking[k] = (bestPackingValueK, bestPackingItemsK, bestPackingCostK)
                        swappedNode = item

        # if not feasibleSwapFound:
        #     return packingHeuristic(clusters, packingValues, service_cost, profits, nodes, m,
        #                                 tabuTag, iteration, alfa, beta, currentSolution, forceSwap)
        # print("aia")
        # minDistance = math.inf
        # bestPacking = {}
        #
        # for k in toPossibleClusters:
        #     if k != c:
        #         centroidForCluster = np.zeros(2)
        #         for j in clusters[k]:
        #             centroidForCluster += coordinates[j]
        #         centroidForCluster /= len(clusters[k])
        #         tmp = np.linalg.norm(coordinates[item] - centroidForCluster)
        #
        #         if tmp < minDistance:
        #             minDistance = tmp
        #             toCluster = k
        #
        # clusteringDict = {c: clusters[c].copy(), toCluster: clusters[toCluster].copy()}
        # clusteringDict[toCluster][item] = clusteringDict[c][item]
        # del clusteringDict[c][item]
        #
        # if not feasibleSwapC:
        #     bestPackingValueC = packingValues[c]
        #
        # if bestPackingProfitFound < bestPackingValueC + packingValues[toCluster]:
        #     bestPacking = {}
        #     partition = clusteringDict.copy()
        #
        #     bestPackingProfitFound = bestPackingValueC + packingValues[toCluster]
        #     bestPacking[c] = (bestPackingValueC, bestPackingItemsC, bestPackingCostC)
        #     bestPacking[toCluster] = (packingValues[toCluster], packingItems[toCluster], packingCosts[toCluster])
        #     swappedNode = item

        i += 1

    # if feasibleImprovement:

    if not feasibleSwapFound:
        return packingHeuristic(clusters, packingValues, service_cost, profits, nodes, m,
                                tabuTag, iteration, alfa, beta, currentSolution, forceSwap)
    else:
        return partition, bestPacking, swappedNode


def clusterizeData(algorithm):
    clusters = []

    if algorithm == 0:
        clustering = AgglomerativeClustering(n_clusters=m, linkage="complete").fit(
            list(coordinates.values())[1:nodes - 1])
    elif algorithm == 1:
        clustering = KMeans(n_clusters=m, random_state=0).fit(list(coordinates.values())[1:nodes - 1])
    elif algorithm == 2:
        clustering = KMedoids.KMedoids(n_clusters=m, random_state=0, init="k-medoids++").fit(
            list(coordinates.values())[1:nodes - 1])
    elif algorithm == 3:
        brc = Birch(n_clusters=m, threshold=0.000001)
        brc.fit(list(coordinates.values())[1:nodes - 1])
        clustering = type('obj', (object,), {'labels_': brc.predict(list(coordinates.values())[1:nodes - 1])})
    elif algorithm == 4:
        clustering = MiniBatchKMeans(n_clusters=m, random_state=0, batch_size=5).fit(
            list(coordinates.values())[1:nodes - 1])
    elif algorithm == 5:
        clustering = SpectralClustering(
            n_clusters=m, assign_labels="discretize", random_state=0).fit(list(coordinates.values())[1:nodes - 1])

    for i in range(0, m):
        clusters.append({})

    for i in range(1, nodes - 1):
        clusters[clustering.labels_[i - 1]][i] = coordinates[i]

    return clusters


def getFeasibleSolutionLKH(distanceMatrix, sortedNodes, travellingTimesMatrix, end, Tmax, s):
    # print(sortedNodes)
    sol_list = elkai.solve_int_matrix(distanceMatrix)

    return_list = []
    cost = 0

    if len(sol_list) > 1:
        for i in range(0, len(sol_list) - 1):
            return_list.append(sortedNodes[sol_list[i]])
            cost = cost + (travellingTimesMatrix[sortedNodes[sol_list[i]]][sortedNodes[sol_list[i + 1]]])

        return_list.append(sortedNodes[sol_list[-1]])
        return_list.append(end)
        cost += (travellingTimesMatrix[sortedNodes[sol_list[-1]]][end] + (Tmax - s))

        return cost, return_list
    else:
        return travellingTimesMatrix[start][end], [start, end]


def getFeasibleSolution(distanceMatrix, sortedNodes, travellingTimesMatrix, start, end, Tmax, s):
    manager = pywrapcp.RoutingIndexManager(len(distanceMatrix), 1, start)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distanceMatrix[from_node][to_node]

    routing = pywrapcp.RoutingModel(manager)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)

    sol = routing.SolveWithParameters(search_parameters)
    if sol:
        sol_list = get_solution(routing, sol)
        return_list = []

        if len(sol_list) > 1:
            cost = 0
            index = 0
            for i in range(0, len(sol_list) - 1):
                return_list.append(sortedNodes[sol_list[i]])
                index = index + 1
                cost = cost + (travellingTimesMatrix[sortedNodes[sol_list[i]]][sortedNodes[sol_list[i + 1]]])

            # cost += (travellingTimesMatrix[sortedNodes[sol_list[index - 1]]][sortedNodes[sol_list[index]]])

            return_list.append(sortedNodes[sol_list[index]])
            return_list.append(end)
            print("RETURN_LIST " + str(return_list))
            return cost + travellingTimesMatrix[sortedNodes[sol_list[index]]][end] + (Tmax - s), return_list
        else:
            return travellingTimesMatrix[start][end], [start, end]
    else:
        return -1, []


def get_solution(routing, solution):
    index = routing.Start(0)
    # route_distance = 0
    sol = []
    while not routing.IsEnd(index):
        sol.append(index)
        # previous_index = index
        index = solution.Value(routing.NextVar(index))
        # route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    return sol


def greedy_aux(start, end, travellingTimesMatrix, cluster, profits, service_cost, Tmax):
    packing = calculatePacking(cluster, service_cost, profits, Tmax)
    packed_items = packing[1]
    total_weight = packing[2]

    tour = calculateTourLKH(Tmax, total_weight, packed_items, start, end, travellingTimesMatrix, service_cost, profits)

    return tour[0], tour[1]


def greedy(clusters):
    p = visitedPlaces = collectedSwabs = 0
    solution = []

    for i in range(0, m):
        info = greedy_aux(0, nodes - 1, travellingTimesMatrix,
                          list(clusters[i].keys()), profits, service_cost, tmax)

        solution.append(info[1])

        p += info[0]
        visitedPlaces += len(info[1]) - 2

        cost = 0
        for place in info[1][1:len(info[1]) - 1]:
            cost = cost + service_cost[place]

        cost1 = 0
        for i in range(0, len(info[1]) - 1):
            cost1 = cost1 + (travellingTimesMatrix[info[1][i]][info[1][i + 1]])

        for place in info[1][1:len(info[1]) - 1]:
            collectedSwabs += nodesInfo[place][0]
    return [p, visitedPlaces, collectedSwabs, solution]


if len(sys.argv) > 2:
    random.seed()
    f = open(str(sys.argv[1]), "r")
    params = open(str(sys.argv[3]), "r")

    parameters = []
    nodesInfo = []
    travellingTimesMatrix = []
    coordinates = {}
    profits = {}
    service_cost = {}
    bestSolutions = []
    tabuTagIn = {}
    tabuTagOut = {}

    RESTARTED = False
    BEST_SOLUTION = -math.inf
    CURRENT_SOLUTION = []
    BEST_SOLUTIONS = []
    NUMBER_OF_NOT_IMPROVING_SOLUTIONS = 0
    FORCE_SWAP_SOLUTION_NODES = False
    FAST_HEURISTIC = False
    PACKINGPROFITS = []
    PACKINGITEMS = []
    TOURPROFITS = []
    TOTALCOSTS = []
    PACKINGCOSTS = []
    NODES_IN_SOLUTION = []

    for i in range(0, 7):
        s = f.readline()
        s = s.replace(';', '')
        s = s[s.index('=') + 1: len(s)]
        parameters.append(int(s))

    nodes = parameters[0]
    tDress = parameters[1]
    tSwab = parameters[2]
    tmax = parameters[5]
    m = parameters[6]

    param = params.readline()
    ALFA = float(param[param.index('=') + 1: len(param)])
    param = params.readline()
    BETA = float(param[param.index('=') + 1: len(param)])
    param = params.readline()
    FRACTION_NODES_TO_SWAP = float(param[param.index('=') + 1: len(param)])
    param = params.readline()
    BLOCK_NODE_ITERATIONS = int(param[param.index('=') + 1: len(param)])
    param = params.readline()
    RESTART_IN_ITERATIONS = int(param[param.index('=') + 1: len(param)])
    param = params.readline()
    FORCE_SWAP_IN_ITERATIONS = int(param[param.index('=') + 1: len(param)])
    param = params.readline()
    INTRA_CLUSTER_OPTIMIZATION_IN_ITERATIONS = int(param[param.index('=') + 1: len(param)])
    param = params.readline()
    MAX_BLOCK_INSERTING_NODE_ITERATIONS_FOR_INTRAOPT = int(param[param.index('=') + 1: len(param)])
    param = params.readline()
    MAX_BLOCK_EXTRACT_NODE_ITERATIONS_FOR_INTRAOPT = int(param[param.index('=') + 1: len(param)])
    param = params.readline()
    MAX_ITERATIONS_WITHOUT_IMPROVEMENT = int(param[param.index('=') + 1: len(param)])
    param = params.readline()
    SWAP_HEURISTIC_IN_ITERATIONS = int(param[param.index('=') + 1: len(param)])

    print("MINIMUM_FRACTION: " + str(ALFA))
    print("MAXIMUM_FRACTION: " + str(BETA))
    print("FRACTION_NODES_TO_SWAP: " + str(FRACTION_NODES_TO_SWAP))
    print("BLOCK_NODE_ITERATIONS: " + str(BLOCK_NODE_ITERATIONS))
    print("RESTART_IN_ITERATIONS: " + str(RESTART_IN_ITERATIONS))
    print("FORCE_SWAP_IN_ITERATIONS: " + str(FORCE_SWAP_IN_ITERATIONS))
    print("RESTART_IN_ITERATIONS: " + str(RESTART_IN_ITERATIONS))
    print("INTRA_CLUSTER_OPTIMIZATION_IN_ITERATIONS: " + str(INTRA_CLUSTER_OPTIMIZATION_IN_ITERATIONS))
    print("MAX_BLOCK_INSERTING_NODE_ITERATIONS_FOR_INTRAOPT: " + str(MAX_BLOCK_INSERTING_NODE_ITERATIONS_FOR_INTRAOPT))
    print("MAX_BLOCK_EXTRACT_NODE_ITERATIONS_FOR_INTRAOPT: " + str(MAX_BLOCK_EXTRACT_NODE_ITERATIONS_FOR_INTRAOPT))

    f.readline()
    for i in range(0, nodes):
        s = f.readline()
        x = float(s[0:s.index(';')])
        s = s[s.index(';') + 1:len(s)]
        y = float(s[0:s.index(';')])
        s = s[s.index(';') + 1:len(s)]
        swabs = s[0:s.index(';')]
        s = s[s.index(';') + 1:len(s)]
        score = s[0:s.index(';')]
        s = s[s.index(';') + 1:len(s)]
        nodesInfo.append([int(swabs), int(score)])
        coordinates[i] = np.array([x, y])

        travellingT = []
        for j in range(0, nodes - 1):
            travellingT.append(int(s[0:s.index(';')]))
            s = s[s.index(';') + 1:len(s)]

        travellingT.append(int(s))
        travellingTimesMatrix.append(travellingT)

    clusters = clusterizeData(int(sys.argv[2]))

    for j in range(0, m):
        tabuTagIn[j] = {}
        tabuTagOut[j] = {}

        for i in range(1, nodes - 1):
            tabuTagIn[j][i] = -1
            tabuTagOut[j][i] = -1

    i = 0
    for cardinality, score in nodesInfo:
        profits[i] = cardinality * score
        service_cost[i] = (2 * tDress + cardinality * tSwab)
        i = i + 1

    [p, visitedPlaces, collectedSwabs, solution] = greedy(clusters)
    totalProfit = GREEDY_SOLUTION = BEST_SOLUTION = p
    print(p)
    print(solution)

    for i in range(0, m):
        pp = calculatePacking(list(clusters[i].keys()), service_cost, profits, tmax)
        # PACKINGPROFITS.append(pp[0])
        # PACKINGITEMS.append(pp[1])
        tourInfo = calculateTourLKH(tmax, pp[2], pp[1], 0, nodes - 1, travellingTimesMatrix, service_cost, profits)
        TOURPROFITS.append(tourInfo[0])
        TOTALCOSTS.append(tourInfo[2])
        CURRENT_SOLUTION.append(tourInfo[1])

        total_weight = 0
        for el in CURRENT_SOLUTION[i][1:-1]:
            total_weight += service_cost[el]

        PACKINGCOSTS.append(total_weight)

        for node in tourInfo[1][1: -1]:
            NODES_IN_SOLUTION.append(node)

    iteration = 0
    now = datetime.now()

    while iteration < 50000:
        item = None
        clusterIndex = None
        nodeSwapped = False
        nodeAdded = False
        gap = -math.inf

        if NUMBER_OF_NOT_IMPROVING_SOLUTIONS % 20 == 0 and NUMBER_OF_NOT_IMPROVING_SOLUTIONS > 0:
            NUMBER_OF_NOT_IMPROVING_SOLUTIONS = 0
            nodesDeleted = destroyTours(CURRENT_SOLUTION, TOTALCOSTS, TOURPROFITS, PACKINGCOSTS,
                                        service_cost, profits,
                                        travellingTimesMatrix, totalProfit)

            for node, i in nodesDeleted:
                NODES_IN_SOLUTION.remove(node)
                tabuTagIn[i][node] = iteration + 15

        for node in range(1, nodes - 1):
            if node not in NODES_IN_SOLUTION:
                for i in range(0, m):
                    if iteration > tabuTagIn[i][node]:
                        move = increaseTour(tmax, CURRENT_SOLUTION[i], TOTALCOSTS[i], TOURPROFITS[i], PACKINGCOSTS[i],
                                            travellingTimesMatrix, service_cost, profits, node)

                        if move is None:
                            move = modifyTour(tmax, CURRENT_SOLUTION[i], TOTALCOSTS[i], TOURPROFITS[i], PACKINGCOSTS[i],
                                              travellingTimesMatrix, service_cost, profits, tabuTagOut, node, i)

                            if move is not None:
                                if gap < move[5]:
                                    gap = move[5]
                                    nodeSwapped = True
                                    nodeAdded = False
                                    deletedNode = move[2]
                                    bestMoveProfit = move[1]
                                    bestMove = move[0]
                                    newCost = move[3]
                                    newPackingCost = move[4]
                                    clusterIndex = i
                                    item = node

                        else:
                            if gap < move[4]:
                                gap = move[4]
                                nodeAdded = True
                                nodeSwapped = False
                                bestMoveProfit = move[1]
                                bestMove = move[0]
                                newCost = move[2]
                                newPackingCost = move[3]
                                clusterIndex = i
                                item = node

        if nodeAdded or nodeSwapped:
            NODES_IN_SOLUTION.append(item)
            CURRENT_SOLUTION[clusterIndex] = bestMove
            TOTALCOSTS[clusterIndex] = newCost
            TOURPROFITS[clusterIndex] = bestMoveProfit
            PACKINGCOSTS[clusterIndex] = newPackingCost

            # print("--- " + str(CURRENT_SOLUTION[clusterIndex][1:-1]))
            # print("+++ " + str(tourInfo[1][1:-1]))

            # CURRENT_SOLUTION[clusterIndex] = tourInfo[1]
            # TOTALCOSTS[clusterIndex] = tourInfo[2]
            # TOURPROFITS[clusterIndex] = tourInfo[0]

            tabuTagOut[clusterIndex][item] = iteration + 10

            # print("Entra " + str(item))

            if nodeSwapped:
                NODES_IN_SOLUTION.remove(deletedNode)
                tabuTagIn[clusterIndex][deletedNode] = iteration + 15
                # print("Esce " + str(deletedNode))

            # print(NODES_IN_SOLUTION)
            # print(CURRENT_SOLUTION)

            totalProfit = 0
            for i in range(0, m):
                totalProfit += TOURPROFITS[i]

            if BEST_SOLUTION < totalProfit:
                BEST_SOLUTION = totalProfit
                NUMBER_OF_NOT_IMPROVING_SOLUTIONS = 0
            else:
                NUMBER_OF_NOT_IMPROVING_SOLUTIONS += 1

            # print(totalProfit)

        # else:
        #     NUMBER_OF_NOT_IMPROVING_SOLUTIONS += 1
        #     # print(iteration)
        #     # print(tabuTagIn)
        #     # exit()

        iteration += 1

    end = datetime.now()
    print((end - now))
    print(BEST_SOLUTION)

    # for i in range(0, m):
    #     pp = calculatePacking(list(clusters[i].keys()), service_cost, profits, tmax)
    #     PACKINGPROFITS.append(pp[0])
    #     PACKINGITEMS.append(pp[1])
    #     PACKINGCOSTS.append(pp[2])
    #     tourInfo = calculateTour(tmax, pp[2], pp[1], 0, nodes - 1, travellingTimesMatrix, service_cost, profits)
    #     TOURPROFITS.append(tourInfo[0])
    #     TOTALCOSTS.append(tourInfo[2])
    #     CURRENT_SOLUTION.append(tourInfo[1])

    # while NUMBER_OF_NOT_IMPROVING_SOLUTIONS < MAX_ITERATIONS_WITHOUT_IMPROVEMENT:

    print(CURRENT_SOLUTION)
    print("Fine ricerca locale.")
    # print(BEST_SOLUTION)
    # # print(NUMBER_OF_TIMES_NODE_IN_SOLUTION)
    # print(max(BEST_SOLUTIONS.keys()))
    # print(BEST_SOLUTIONS[max(BEST_SOLUTIONS.keys())])

    # print(BEST_SOLUTIONS[sortedSolutions[0]][0])
    # plotClusters(BEST_SOLUTIONS[sortedSolutions[0]][1], nodes, coordinates, BEST_SOLUTIONS[sortedSolutions[0]][0])
