import numpy as np
import math
import csv

pt = list[float] | tuple[float, float]

def distManhattan(pt1: pt, pt2: pt) -> int:
    return abs(pt1[0] - pt2[0]) + abs(pt1[1] - pt2[1])

def distEuclidean(pt1: pt, pt2: pt) -> float:
    return math.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)

class OutOfRangeError(Exception):
    pass

def loadVertiportsInfo(filePath: str) -> dict:
    # 初始化字典
    vertiports = {}

    with open(filePath, 'r', encoding='utf-8') as file:
        reader = csv.DictReader(file) # 以列名为键读取
        for row in reader:
            v = int(row['站点序'])
            X = float(row['X'])
            Y = float(row['Y'])

            vertiports[v] = {'loc': [X, Y]}

    return vertiports

def loadOrdersInfo(filePath: str, OrderNum: int) -> dict:
    # 初始化字典
    orders = {}
    i = 0

    with open(filePath, 'r', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        for row in reader:
            OrderID = int(row['订单序'])
            readyTime = float(row['Ready Time'])
            dueTime = float(row['Due Date'])
            revenue = float(row['Order_Revenue'])
            O_X = float(row['O_X'])
            O_Y = float(row['O_Y'])
            D_X = float(row['D_X'])
            D_Y = float(row['D_Y'])

            orders[OrderID] = {
                'readyTime': readyTime,
                'dueTime': dueTime,
                'revenue': revenue,
                'pickupLoc': [O_X, O_Y],
                'deliveryLoc': [D_X, D_Y]
            }

            i += 1
            if i == OrderNum:
                break

    return orders

def loadDronesInfo(droneNum: int, vertInit: int, StartTime: float, EndTime: float) -> dict:
    drones = {
        u: {
            'vertInit': vertInit,
            'availStartTime': StartTime,
            'availEndTime': EndTime
        } for u in range(1, droneNum + 1)
    }

    return drones