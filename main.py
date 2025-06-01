from vrptw import *

if __name__ == "__main__":
    orders = loadOrdersInfo('KDLSA-SS-CG/BADS/BADS-11/BADS-1101.csv', CONST_ORDER_NUM)
    vertiports = loadVertiportsInfo('KDLSA-SS-CG/Drone Net Information.csv')
    drones = loadDronesInfo(
        droneNum = CONST_DRONE_NUM,
        vertInit = CONST_DRONE_ID,
        StartTime = CONST_EARLIEST_TIME,
        EndTime = COSNT_LATEST_TIME
    )
    sol = solveVRPTW(orders, vertiports, drones)
    print(sol)