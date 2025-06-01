import gurobipy as grb
import datetime
import numpy as np

from common import *
from const import *
from data import *

def solveVRPTW(orders: dict, vertiports: dict, drones: dict) -> dict:
    # 首先构造漫游集
    orderTFTemp = {}
    orderTF = {}
    for o in orders:
        orderTFTemp[o] = {}
        inRangeV = [] # 地面配送范围内的vertiport
        for v in vertiports:
            # NOTE: 地面配送走曼哈顿距离
            distToTF = distManhattan(
                pt1 = orders[o]['pickupLoc'],
                pt2 = vertiports[v]['loc']
            )

            # 判断是否在地面可配送范围内, 若可以配送, 递推无人机最早起飞时间和最晚起飞时间, 以及配送耗电和配送时间
            if (distToTF <= CONST_TAKEOF_SELECTION_RANGE):
                inRangeV.append(v)
                startLoc = vertiports[v]['loc']
                endLoc = orders[o]['deliveryLoc']
                distToCus = distEuclidean(startLoc, endLoc)
                if (distToCus <= CONST_MAX_RANGE - CONST_EPSILON):
                    timeToTF = distToTF / CONST_TRUCK_DELIVERY_SPEED
                    estTF = orders[o]['readyTime'] + timeToTF + CONST_LOAD_TIME
                    timeToCus = distToCus / CONST_DRONE_SPEED
                    latTF = orders[o]['dueTime'] - timeToCus - CONST_LOAD_TIME
                    ergyToCus = distToCus # 耗电以里程计

                    orderTFTemp[o][v] = {
                        'estTF': estTF,
                        'latTF': latTF,
                        'distToTF': distToTF,
                        'timeToTF': timeToTF,
                        'distToCus': distToCus,
                        'timeToCus': timeToCus,
                        'ergyToCus': ergyToCus
                    }

        if len(inRangeV) == 0:
            raise OutOfRangeError(f'ERROR: OrderID {o} cannot be delivered due to ground delivery range.')
        
        if len(orderTFTemp[o]) == 0:
            raise OutOfRangeError(f'ERROR: OrderID {o} cannot be delivered due to drone battery.')
    
    for o in orders:
        for v in orderTFTemp[o]:
            orderTF[o, v] = orderTFTemp[o][v]

    orderSuccTemp = {}
    orderSucc = {}

    for o in orders:
        orderSuccTemp[o] = {}

        startLoc = orders[o]['deliveryLoc']
        for vNext in vertiports:
            orderSuccTemp[o][vNext] = {}
            endLoc = vertiports[vNext]['loc']
            distToVNext = distEuclidean(startLoc, endLoc)
            timeToVNext = distToVNext / CONST_DRONE_SPEED
            ergy = distToVNext

            # NOTE: 跳过无人机到不了的
            if ergy >= CONST_MAX_RANGE + CONST_EPSILON:
                continue

            for v in orderTFTemp[o]:
                orderSuccTemp[o][vNext][v] = {}
                # o在v处的最早起飞时间
                estTF = orderTF[o, v]['estTF']
                # 从v到vNext的总时间
                timeVToVNext = orderTF[o, v]['timeToCus'] + CONST_LOAD_TIME + timeToVNext

                for oNext in orders:
                    if (
                        oNext != o 
                        and vNext in orderTFTemp[oNext] 
                        and estTF + timeVToVNext + CONST_LOAD_TIME <= orderTF[oNext, vNext]['latTF'] - CONST_EPSILON
                        ):
                        # 在v站点的起飞时刻
                        estTF = estTF
                        latTF = min(
                            orderTF[o, v]['latTF'],
                            orderTF[oNext, vNext]['latTF'] - timeVToVNext - CONST_LOAD_TIME
                        )

                        # 在vNext站点的起飞时刻
                        estTFVNext = max(
                            estTF + timeVToVNext + CONST_LOAD_TIME,
                            orderTF[oNext, vNext]['estTF']
                        )
                        latTFVNext = orderTF[oNext, vNext]['latTF']
                        if estTFVNext <= latTFVNext - CONST_EPSILON:
                            orderSuccTemp[o][vNext][v][oNext] = {
                                'estTF': estTF,
                                'latTF': latTF,
                                'estTFVNext': estTFVNext,
                                'latTFVNext': latTFVNext,
                                'timeVToVNext': timeVToVNext,
                                'ergyToCus': orderTF[o, v]['ergyToCus'],
                                'timeToCus': orderTF[o, v]['timeToCus'],
                                'ergyToVNext': ergy,
                                'timeToVNext': timeToVNext
                            }

    for o in orders:
        for oNext in orders:
            for v in vertiports:
                for vNext in vertiports:
                    if (o in orderSuccTemp and vNext in orderSuccTemp[o] and v in orderSuccTemp[o][vNext] and oNext in orderSuccTemp[o][vNext][v]):
                        orderSucc[o, v, oNext, vNext] = orderSuccTemp[o][vNext][v][oNext]
    
    initialSuccTemp = {}
    initialSucc = {}

    for u in drones:
        initialSuccTemp[u] = {}
        startLoc = vertiports[drones[u]['vertInit']]['loc']

        for v in vertiports:
            initialSuccTemp[u][v] = {}
            if (v != drones[u]['vertInit']):
                endLoc = vertiports[v]['loc']
                distToVert = distEuclidean(startLoc, endLoc)
                timeToVert = distToVert / CONST_DRONE_SPEED

                ergy = distToVert

                if (ergy >= CONST_MAX_RANGE + CONST_EPSILON):
                    continue

                estTF = drones[u]['availStartTime'] + timeToVert + CONST_LOAD_TIME
                for o in orders:
                    if ((o, v) in orderTF and estTF <= orderTF[o, v]['latTF'] - CONST_EPSILON):
                        initialSuccTemp[u][v][o] = {
                            'estTF': estTF,
                            'ergyToVert': ergy,
                            'timeToVert': timeToVert
                        }
            else:
                for o in orders:
                    if ((o, v) in orderTF and drones[u]['availStartTime'] <= orderTF[o, v]['latTF'] - CONST_EPSILON):
                        initialSuccTemp[u][v][o] = {
                            'estTF': drones[u]['availStartTime'],
                            'ergyToVert': 0,
                            'timeToVert': 0
                        }
    
    for u in drones:
        for v in vertiports:
            for o in orders:
                if (u in initialSuccTemp and v in initialSuccTemp[u] and o in initialSuccTemp[u][v]):
                    initialSucc[o, v, u] = initialSuccTemp[u][v][o]

    returnSuccTemp = {}
    returnSucc = {}

    for v in vertiports:
        returnSuccTemp[v] = {}

        for o in orders:
            returnSuccTemp[v][o] = {}

            if (v not in orderTFTemp[o]):
                continue

            ergyToCus = orderTF[o, v]['ergyToCus']
            timeToCus = orderTF[o, v]['timeToCus']
            startLoc = orders[o]['deliveryLoc']

            for u in drones:
                endLoc = vertiports[drones[u]['vertInit']]['loc']
                distToBase = distEuclidean(startLoc, endLoc)
                timeToBase = distToBase / CONST_DRONE_SPEED

                ergy = distToBase

                if (ergy >= CONST_MAX_RANGE + CONST_EPSILON):
                    continue
                
                returnSuccTemp[v][o][u] = {
                    'timeVToBase': timeToCus + CONST_LOAD_TIME + timeToBase,
                    'ergyToCus': ergyToCus,
                    'timeToCus': timeToCus,
                    'ergyToBase': ergy,
                    'timeToBase': timeToBase
                }               

    for v in vertiports:
        for o in orders:
            for u in drones:
                if (v in returnSuccTemp and o in returnSuccTemp[v] and u in returnSuccTemp[v][o]):
                    returnSucc[o, v, u] = returnSuccTemp[v][o][u]
    
    # 定价子问题 ===============================================================
    # pi['orders'][o] = pi_o
    # pi['drones'][drone] = pi_drone

    def _pricingLabelSetting(pi):
        def _initializeLabel():
            ordersIC = set()
            ordersPD = set() 
            ordersPDInfo = {} # 后续拓展可行订单相关信息
            for (o, v, u) in initialSucc:
                if (u != 1):
                    continue
                
                swapAtCus = 0 # oNext
                timeAtVert = max(drones[1]['availStartTime'] + initialSucc[o, v, u]['timeToVert'] + CONST_LOAD_TIME, orderTF[o, v]['estTF'])
                timeAtCus = timeAtVert + orderTF[o, v]['timeToCus'] + CONST_LOAD_TIME

                if timeAtCus + returnSucc[o, v, u]['timeToBase'] <= drones[1]['availEndTime'] - CONST_EPSILON:
                    ergyAtVert = CONST_MAX_RANGE - initialSucc[o, v, u]['ergyToVert']
                    if ergyAtVert < orderTF[o, v]['ergyToCus'] - CONST_EPSILON:
                        # NOTE: oNext  
                        swapAtVert = 1
                        ergyAtCus = CONST_MAX_RANGE - orderTF[o, v]['ergyToCus']
                    else:
                        swapAtVert = 0
                        ergyAtCus = ergyAtVert - orderTF[o, v]['ergyToCus']
                    costs = CONST_SWAP_COST * (swapAtCus + swapAtVert) + CONST_RIDER_COST * orderTF[o, v]['timeToTF'] - orders[o]['revenue'] - pi['orders'][o]

                    ordersPD.add((o, v))
                    ordersPDInfo[o, v] = {
                        'swapAtCus': swapAtCus, # NOTE: 在中心机巢处("前序订单"的customer处)
                        'swapAtVert': swapAtVert,
                        'timeAtVert': timeAtVert,
                        'timeAtCus': timeAtCus,
                        'ergyAtVert': ergyAtVert,
                        'ergyAtCus': ergyAtCus,
                        'costs': costs
                    }
                

            return {
                'path': [(0, drones[1]['vertInit'])],
                'swapSum': 0, # 当前总换电次数
                'swapAtVert': 0, # vertiport处是否换电
                'swapAtCus': 0, # customer处是否换电
                'ergyAtVert': CONST_MAX_RANGE, # vertiport处剩余电量
                'ergyAtCus': CONST_MAX_RANGE, # vustomer处剩余电量
                'timeAtVert': drones[1]['availStartTime'], # vertiport处时间戳
                'timeAtCus': drones[1]['availStartTime'], # customer处时间戳
                'RC': CONST_FIXED_COST + CONST_SWAP_COST - pi['drones'][1], # 检验数
                'ordersIC': ordersIC, # orders included
                'ordersPD': ordersPD, # orders pending
                'ordersPDInfo': ordersPDInfo
            }
        
        def _feasibility(curPath, keyOrders):
            # 这里的参数curPath即为label, 只不过少了'ordersPD'和'ordersPDInfo', _feasibility()就是要求这两个
            ordersPD = set()
            ordersPDInfo = {}

            for (o, v, oNext, vNext) in orderSucc:
                if (o, v) != curPath['path'][-1]:
                    continue

                if oNext in keyOrders:
                    canAddFlag = True
                    for (o1, v1) in curPath['ordersIC']:
                        if o1 == oNext:
                            canAddFlag = False
                            break
                    if not canAddFlag:
                        continue


                if (oNext, vNext) in curPath['ordersIC']:
                    continue
                
                # 配送oNext的时间窗可行性
                if curPath['timeAtVert'] >= orderSucc[o, v, oNext, vNext]['latTF'] + CONST_EPSILON:
                    continue
                
                # 配送oNext的电量可行性
                # (o的customer处)—(vNext)段
                if curPath['ergyAtCus'] >= orderSucc[o, v, oNext, vNext]['ergyToVNext'] + CONST_EPSILON:
                    swapAtCus = 0
                    ergyAtVert = curPath['ergyAtCus'] - orderSucc[o, v, oNext, vNext]['ergyToVNext']
                elif curPath['swapSum'] < CONST_MAX_SWAP:
                    swapAtCus = 1
                    ergyAtVert = CONST_MAX_RANGE - orderSucc[o, v, oNext, vNext]['ergyToVNext']
                else:
                    continue

                # (vNext)-(oNext的customer)段
                if ergyAtVert >= orderTF[oNext, vNext]['ergyToCus'] + CONST_EPSILON:
                        swapAtVert = 0
                        ergyAtCus = ergyAtVert - orderTF[oNext, vNext]['ergyToCus']
                elif curPath['swapSum'] + swapAtCus < CONST_MAX_SWAP:
                    swapAtVert = 1
                    ergyAtCus = CONST_MAX_RANGE - orderTF[oNext, vNext]['ergyToCus']
                else:
                    continue    

                # 配送oNext的返回中心机巢电量可行性
                if not (curPath['swapSum'] + swapAtCus + swapAtVert < CONST_MAX_SWAP or ergyAtCus >= returnSucc[oNext, vNext, 1]['ergyToBase'] + CONST_EPSILON):
                    continue

                # 配送oNext的返回中心机巢时间可行性
                timeAtVert = max(curPath['timeAtCus'] + orderSucc[o, v, oNext, vNext]['timeToVNext'] + CONST_LOAD_TIME, orderTF[oNext, vNext]['estTF'])
                timeAtCus = timeAtVert + orderTF[oNext, vNext]['timeToCus'] + CONST_LOAD_TIME
                if timeAtCus + returnSucc[oNext, vNext, 1]['timeToBase'] >= drones[1]['availEndTime'] + CONST_EPSILON:
                    continue

                # 成本
                costs = CONST_SWAP_COST * (swapAtCus + swapAtVert) + CONST_RIDER_COST * orderTF[oNext, vNext]['timeToTF'] - orders[oNext]['revenue'] - pi['orders'][oNext]

                ordersPD.add((oNext, vNext))
                ordersPDInfo[oNext, vNext] = {
                    'swapAtCus': swapAtCus,
                    'swapAtVert': swapAtVert,
                    'timeAtVert': timeAtVert,
                    'timeAtCus': timeAtCus,
                    'ergyAtVert': ergyAtVert,
                    'ergyAtCus': ergyAtCus,
                    'costs': costs
                }

            return ordersPD, ordersPDInfo
        
        def _extendPath(curPath, keyOrders):
            newPaths = []
            for (o, v) in curPath['ordersPD']:
                path = curPath['path'].copy()
                path.append((o, v))
                
                swapAtCus = curPath['ordersPDInfo'][o, v]['swapAtCus']
                swapAtVert = curPath['ordersPDInfo'][o, v]['swapAtVert']
                swapSum = curPath['swapSum'] + swapAtCus + swapAtVert
                
                ergyAtCus = curPath['ordersPDInfo'][o, v]['ergyAtCus']
                ergyAtVert = curPath['ordersPDInfo'][o, v]['ergyAtVert']
                
                timeAtCus = curPath['ordersPDInfo'][o, v]['timeAtCus']
                timeAtVert = curPath['ordersPDInfo'][o, v]['timeAtVert']

                RC = curPath['RC'] + curPath['ordersPDInfo'][o, v]['costs']

                ordersIC = curPath['ordersIC'].copy()
                ordersIC.add((o, v))

                newPath = {
                    'path': path,
                    'swapSum': swapSum,
                    'swapAtVert': swapAtVert,
                    'swapAtCus': swapAtCus,
                    'ergyAtVert': ergyAtVert,
                    'ergyAtCus': ergyAtCus,
                    'timeAtVert': timeAtVert,
                    'timeAtCus': timeAtCus,
                    'RC': RC,
                    'ordersIC': ordersIC
                }

                ordersPD, ordersPDInfo = _feasibility(newPath, keyOrders)
                newPath['ordersPD'] = ordersPD
                newPath['ordersPDInfo'] = ordersPDInfo

                newPaths.append(newPath)

            return newPaths
        
        def _dominate(label1, label2):
            if(
                label1['path'][-1] == label2['path'][-1]
                and label1['timeAtCus'] <= label2['timeAtCus'] - CONST_EPSILON
                and ((label1['swapSum'] < label2['swapSum']) or ((label1['swapSum'] == label2['swapSum']) and (label1['ergyAtCus'] >= label2['ergyAtCus'] + CONST_EPSILON)))
                and label1['RC'] <= label2['RC'] - CONST_EPSILON
                and label1['ordersIC'] >= label2['ordersIC']
                and label1['ordersPD'] >= label2['ordersPD']
            ):
                return True
            else:
                return False
            
        def _sortQueue(queue):
            queue = sorted(queue, key = lambda d: d['RC'])
            return queue
        
        # Main =============================================================

        # 关键订单集
        keyOrders = []

        while True:
            # 初始label, queue, path
            queue = [_initializeLabel()]

            path = []

            # 主循环
            while(len(queue) > 0):
                # 从队列中取出第一个
                curPath = queue.pop()

                # 取出来的路径是不是非支配解?
                curPathNonDominatedFlag = True
                for i in range(len(queue)):
                    if(_dominate(queue[i], curPath)):
                        curPathNonDominatedFlag = False
                        break
                
                # 如果取出来的路径不是支配解, 则说明有可能可以拓展
                if(curPathNonDominatedFlag):
                    extendPaths = _extendPath(curPath, keyOrders)
                    # 如果可以拓展
                    if (extendPaths): 
                        for extendPath in extendPaths:
                            nonDominatedFlag = True
                            for i in range(len(queue)):
                                if (_dominate(queue[i], extendPath)):
                                    nonDominatedFlag = False
                                    break

                            if nonDominatedFlag:
                                queue.append(extendPath)
                                # print(extendPath)
                        queue = _sortQueue(queue)
                    else:
                        path.append(curPath)

            # 对列表path中的路径进行最后一步拓展, 即返回中心机巢
            # NOTE: 根据标签生成规则, 确保了每个标签对应的路径一定可以返回中心机巢
            for label in path:
                (o, v) = label['path'][-1]
                if (label['ergyAtCus'] < returnSucc[o, v, 1]['ergyToBase'] - CONST_EPSILON):
                    label['swapSum'] += 1
                    label['ergyAtDP'] = CONST_MAX_RANGE - returnSucc[o, v, 1]['ergyToBase'] # 返回中心机巢后的剩余电量
                    label['RC'] += CONST_SWAP_COST
                else:
                    label['ergyAtDP'] = label['ergyAtCus'] - returnSucc[o, v, 1]['ergyToBase'] # 返回中心机巢后的剩余电量
                
                label['timeAtDP'] = label['timeAtCus'] + returnSucc[o, v, 1]['timeToBase']
                label['path'].append((CONST_ORDER_NUM + 1, drones[1]['vertInit']))

            path = _sortQueue(path)
            optPath = path[0]
            
            ordersSum = np.zeros(CONST_ORDER_NUM)
            for (o, v) in optPath['path'][1:-1]:
                ordersSum[o - 1] += 1
            order = np.argmax(ordersSum)
            if ordersSum[order] > 1:
                keyOrders.append(order + 1)
            else:
                break

        C_gamma = optPath['RC']
        alpha = {i: 0 for i in range(1, CONST_ORDER_NUM + 1)}
        for (o, v) in optPath['path'][1:-1]:
            alpha[o] = 1
            C_gamma += pi['orders'][o]
        C_gamma += pi['drones'][1]

        return {
            'C_gammaRC': optPath['RC'],
            'C_gamma': C_gamma,
            'route': optPath['path'],
            'alpha': alpha,
            'swapSum': optPath['swapSum'],
            'ergy': optPath['ergyAtDP']
        }
    
    def _initialSolution():
        ordersRoaming = list(orderTF.keys())
        solutions = []
        
        for _ in range(CONST_DRONE_NUM):
            ordersInitial = []
            for (o, v) in ordersRoaming:
                if (o, v, 1) in initialSucc:
                    ordersInitial.append((o, v))
            # 按照距离最短、时间窗最紧的优先级对订单进行排序
            orderMinTime = sorted(ordersInitial, key = lambda d: (initialSucc[d[0], d[1], 1]['timeToVert'], orderTF[d]['latTF']))
            (o, v) = (0, drones[1]['vertInit'])
            s = drones[1]['availStartTime']
            ergy = CONST_MAX_RANGE
            h = CONST_MAX_SWAP
            path = []
            path.append((o, v))
            C_gamma = CONST_FIXED_COST + CONST_SWAP_COST
            sol = {}

            while ordersRoaming and ergy > CONST_EPSILON and s < drones[1]['availEndTime'] - CONST_EPSILON and orderMinTime:
                (oNext, vNext) = orderMinTime.pop(0)
                if o == 0:
                    # 时间窗可行性
                    if (s + initialSucc[oNext, vNext, 1]['timeToVert'] + CONST_LOAD_TIME <= orderTF[oNext, vNext]['latTF'] - CONST_EPSILON) and (s + initialSucc[oNext, vNext, 1]['timeToVert'] + CONST_LOAD_TIME * 2 + orderTF[oNext, vNext]['timeToCus'] + returnSucc[oNext, vNext, 1]['timeToBase'] <= drones[1]['availEndTime'] - CONST_EPSILON):
                        
                        # 电量可行性
                        ergyToVertFlag = False
                        ergyToCusFlag = False
                        ergyReturnFlag = False
                        if ergy >= initialSucc[oNext, vNext, 1]['ergyToVert'] + CONST_EPSILON:
                            swapAtCus = 0
                            ergyAtVert = ergy - initialSucc[oNext, vNext, 1]['ergyToVert']
                            ergyToVertFlag = True
                        elif h > 0:
                            swapAtCus = 1
                            ergyAtVert = CONST_MAX_RANGE - initialSucc[oNext, vNext, 1]['ergyToVert']
                            ergyToVertFlag = True
                        
                        if ergyToVertFlag:
                            if ergyAtVert >= orderTF[oNext, vNext]['ergyToCus'] + CONST_EPSILON:
                                swapAtVert = 0
                                ergyAtCus = ergyAtVert - orderTF[oNext, vNext]['ergyToCus']
                                ergyToCusFlag = True
                            elif h - swapAtCus > 0:
                                swapAtVert = 1
                                ergyAtCus = CONST_MAX_RANGE - orderTF[oNext, vNext]['ergyToCus']
                                ergyToCusFlag = True
                        
                        if ergyToCusFlag:
                            if (ergyAtCus >= returnSucc[oNext, vNext, 1]['ergyToBase'] + CONST_EPSILON) or (h - swapAtCus - swapAtVert > 0):
                                ergyReturnFlag = True

                        # 资源拓展
                        if ergyReturnFlag:
                            ordersRoaming = list(filter(lambda x: x[0] != oNext, ordersRoaming))
                            path.append((oNext, vNext))
                            ergy = ergyAtCus
                            h = h - swapAtCus - swapAtVert
                            timeAtVert = max(s + initialSucc[oNext, vNext, 1]['timeToVert'] + CONST_LOAD_TIME, orderTF[oNext, vNext]['estTF'])
                            s = timeAtVert + orderTF[oNext, vNext]['timeToCus'] + CONST_LOAD_TIME
                            C_gamma = C_gamma + (swapAtCus + swapAtVert) * CONST_SWAP_COST - (orders[oNext]['revenue'] - orderTF[oNext, vNext]['timeToTF'] * CONST_RIDER_COST)
                            (o, v) = (oNext, vNext)
                            ordersNext = []
                            for (o1, v1, o2, v2) in orderSucc:
                                if (o1, v1) == (o, v) and (o2, v2) in ordersRoaming:
                                    ordersNext.append((o2, v2))
                            orderMinTime = sorted(ordersNext, key = lambda d: (orderSucc[o, v, d[0], d[1]]['timeToVNext'], orderTF[d]['latTF']))
                else:
                    # 时间窗可行性
                    if (s + orderSucc[o, v, oNext, vNext]['timeToVNext'] + CONST_LOAD_TIME <= orderTF[oNext, vNext]['latTF'] - CONST_EPSILON) and (s + orderSucc[o, v, oNext, vNext]['timeToVNext'] + CONST_LOAD_TIME * 2 + orderTF[oNext, vNext]['timeToCus'] + returnSucc[oNext, vNext, 1]['timeToBase'] <= drones[1]['availEndTime'] - CONST_EPSILON):
                        
                        # 电量可行性
                        ergyToVertFlag = False
                        ergyToCusFlag = False
                        ergyReturnFlag = False
                        if ergy >= orderSucc[o, v, oNext, vNext]['ergyToVNext'] + CONST_EPSILON:
                            swapAtCus = 0
                            ergyAtVert = ergy - orderSucc[o, v, oNext, vNext]['ergyToVNext']
                            ergyToVertFlag = True
                        elif h > 0:
                            swapAtCus = 1
                            ergyAtVert = CONST_MAX_RANGE - orderSucc[o, v, oNext, vNext]['ergyToVNext']
                            ergyToVertFlag = True
                        
                        if ergyToVertFlag:
                            if ergyAtVert >= orderTF[oNext, vNext]['ergyToCus'] + CONST_EPSILON:
                                swapAtVert = 0
                                ergyAtCus = ergyAtVert - orderTF[oNext, vNext]['ergyToCus']
                                ergyToCusFlag = True
                            elif h - swapAtCus > 0:
                                swapAtVert = 1
                                ergyAtCus = CONST_MAX_RANGE - orderTF[oNext, vNext]['ergyToCus']
                                ergyToCusFlag = True
                        
                        if ergyToCusFlag:
                            if (ergyAtCus >= returnSucc[oNext, vNext, 1]['ergyToBase'] + CONST_EPSILON) or (h - swapAtCus - swapAtVert > 0):
                                ergyReturnFlag = True

                        if ergyReturnFlag:
                            ordersRoaming = list(filter(lambda x: x[0] != oNext, ordersRoaming))
                            path.append((oNext, vNext))
                            ergy = ergyAtCus
                            h = h - swapAtCus - swapAtVert
                            timeAtVert = max(s + orderSucc[o, v, oNext, vNext]['timeToVNext'] + CONST_LOAD_TIME, orderTF[oNext, vNext]['estTF'])
                            s = timeAtVert + orderTF[oNext, vNext]['timeToCus'] + CONST_LOAD_TIME
                            C_gamma = C_gamma + (swapAtCus + swapAtVert) * CONST_SWAP_COST - (orders[oNext]['revenue'] - orderTF[oNext, vNext]['timeToTF'] * CONST_RIDER_COST)
                            (o, v) = (oNext, vNext)
                            ordersNext = []
                            for (o1, v1, o2, v2) in orderSucc:
                                if (o1, v1) == (o, v) and (o2, v2) in ordersRoaming:
                                    ordersNext.append((o2, v2))
                            orderMinTime = sorted(ordersNext, key = lambda d: (orderSucc[o, v, d[0], d[1]]['timeToVNext'], orderTF[d]['latTF']))

            if ergy < returnSucc[o, v, 1]['ergyToBase'] - CONST_EPSILON:
                C_gamma += CONST_SWAP_COST
                h -= 1
                ergy = CONST_MAX_RANGE - returnSucc[o, v, 1]['ergyToBase']
            else:
                ergy -= returnSucc[o, v, 1]['ergyToBase']

            path.append((CONST_ORDER_NUM + 1, drones[1]['vertInit']))
            alpha = {i: 0 for i in range(1, CONST_ORDER_NUM + 1)}
            for (o, v) in path[1:-1]:
                alpha[o] = 1
            
            sol['C_gamma'] = C_gamma
            sol['route'] = path
            sol['alpha'] = alpha
            sol['swapSum'] = CONST_MAX_SWAP - h
            sol['ergy'] = ergy

            solutions.append(sol)

        return solutions
    
    # Master problem ===========================================
    VRPTW = grb.Model('VRPTW')
    VRPTW.setParam('OutputFlag', 0)

    routes = {}
    c = {} # 路径成本, c[routeID]
    alpha = {} # 订单是否在路径中, alpha[orderID, routeID] 
    swapSum = {}
    ergy = {}

    startTime = datetime.datetime.now()
    # 初始解
    initialSolution = _initialSolution()
    for routeID in range(1, len(initialSolution) + 1):
        routes[routeID] = initialSolution[routeID - 1]['route']
        c[routeID] = initialSolution[routeID - 1]['C_gamma']
        swapSum[routeID] = initialSolution[routeID - 1]['swapSum']
        ergy[routeID] = initialSolution[routeID - 1]['ergy']
        for orderID in initialSolution[routeID - 1]['alpha']:
            alpha[orderID, routeID] = initialSolution[routeID - 1]['alpha'][orderID]

    writeLog(hyphenStr(""))
    writeLog(f"InitialRoute found: {routes}")
    writeLog(f"C_gamma: {c}")
    writeLog("time: " + str(round((datetime.datetime.now() - startTime).total_seconds(), 2)) + "[s]")
    writeLog(hyphenStr(""))

    # 决策变量
    z = {}
    for routeID in routes:
        z[routeID] = VRPTW.addVar(vtype=grb.GRB.CONTINUOUS, lb=0.0, ub=1.0, obj=c[routeID])

    # 约束
    cons = {}
    # 订单
    cons['orders'] = {}
    for orderID in range(1, CONST_ORDER_NUM + 1):
        cons['orders'][orderID] = VRPTW.addConstr(grb.quicksum(alpha[orderID, routeID] * z[routeID] for routeID in routes) <= 1)
    # 无人机
    cons['drones'] = VRPTW.addConstr(grb.quicksum(z[routeID] for routeID in z) <= CONST_DRONE_NUM)

    VRPTW.ModelSense = grb.GRB.MINIMIZE
    VRPTW.optimize()

    # 列生成 ===================================================
    canAddVarFlag = True
    Iter = 0
    while(canAddVarFlag):
        canAddVarFlag = False

        startIter = datetime.datetime.now()

        if (VRPTW.status == grb.GRB.status.OPTIMAL):
            # 解子问题
            pi = {}
            pi['orders'] = {}
            pi['drones'] = {}

            for orderID in cons['orders']:
                pi['orders'][orderID] = cons['orders'][orderID].Pi

            pi['drones'][1] = cons['drones'].Pi

            # 将约束的对偶传入给子问题
            subproblem = _pricingLabelSetting(pi)

            # 如果子问题的RC小于0
            if (subproblem['C_gammaRC'] < -CONST_EPSILON):
                canAddVarFlag = True
                Iter += 1
                writeLog(hyphenStr(""))
                writeLog(f"Iteration: {Iter}")
                writeLog(f"Subroute found: {subproblem['route']}")
                writeLog(f"C_gamma: {subproblem['C_gamma']}")
                writeLog(f"C_gammaRC: {subproblem['C_gammaRC']}")
                writeLog("Iteration time: " + str(round((datetime.datetime.now() - startIter).total_seconds(), 2)) + "[s]")

                routeID = max(list(z.keys())) + 1
                c[routeID] = subproblem['C_gamma']
                z[routeID] = VRPTW.addVar(vtype=grb.GRB.CONTINUOUS, lb=0.0, ub=1.0, obj=c[routeID])

                for orderID in subproblem['alpha']:
                    alpha[orderID, routeID] = subproblem['alpha'][orderID]
                routes[routeID] = subproblem['route']
                swapSum[routeID] = subproblem['swapSum']
                ergy[routeID] = subproblem['ergy']

                VRPTW.chgCoeff(cons['drones'], z[routeID], 1)
                for orderID in range(1, CONST_ORDER_NUM + 1):
                    VRPTW.chgCoeff(cons['orders'][orderID], z[routeID], alpha[orderID, routeID])
                VRPTW.update()
            else:
                canAddVarFlag = False
        else:
            canAddVarFlag = False

        VRPTW.optimize()

    Time = round((datetime.datetime.now() - startTime).total_seconds(), 2)

    lb = VRPTW.getObjective().getValue()

    for i in z:
        z[i].vtype = grb.GRB.BINARY
    VRPTW.update()
    VRPTW.optimize()

    solRoute = {}
    acc = 1
    for i in z:
        if(z[i].x > 0.9):
            solRoute[acc] = {
                'route': routes[i],
                'costs': c[i],
                'z': z[i].x,
                'swapSum': swapSum[i],
                'ergy': ergy[i]
            } 
            acc += 1
    ofv = VRPTW.getObjective().getValue()

    return {
        'ofv': ofv,
        'lb': lb,
        'ub': ofv,
        'gap': abs((ofv - lb) / lb),
        'routes': solRoute,
        'Time': Time
    }