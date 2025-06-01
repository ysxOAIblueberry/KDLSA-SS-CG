import random
import math
import datetime
import functools

CONST_EPSILON = 0.00001
CONST_EARTH_RADIUS_MILES = 3958.8
CONST_EARTH_RADIUS_METERS = 6378137.0

# Type alias
pt = list[float] | tuple[float, float]
poly = list[list[float]] | list[tuple[float, float]]
polys = list[list[list[float]]] | list[list[tuple[float, float]]]
circle = tuple[pt, float]
line = list[pt]

DEBUG_WRITE_LOG = False
DEBUG_PRINT_LOG = True
DEBUG_LOG_PATH = "log.log"

class UnsupportedInputError(Exception):
    pass

class ZeroVectorError(Exception):
    pass

class InvalidPolygonError(Exception):
    pass

class EmptyError(Exception):
    pass

class MissingParameterError(Exception):
    pass

class KeyExistError(Exception):
    pass

class KeyNotExistError(Exception):
    pass

class OutOfRangeError(Exception):
    pass

class VrpSolverNotAvailableError(Exception):
    pass

def list2String(l, noCommaFlag=False):
    listString = "["
    if (noCommaFlag==False):
        listString += ', '.join([list2String(elem) if type(elem) == list else str(elem) for elem in l.copy()])
    else:
        listString += ''.join([list2String(elem) if type(elem) == list else str(elem) for elem in l.copy()])
    listString += "]"
    return listString

def list2Tuple(l):
    sortedList = [i for i in l]
    sortedList.sort()
    tp = tuple(sortedList)
    return tp

def hyphenStr(s="", length=75, sym='-'):
    if (s == ""):
        return length * sym
    lenMidS = len(s)
    if (lenMidS + 2 < length):
        lenLeftS = (int)((length - lenMidS - 2) / 2)
        lenRightS = length - lenMidS - lenLeftS - 2
        return (lenLeftS * sym) + " " + s + " " + (lenRightS * sym)
    else:
        return s

def writeLog(string, logPath = None):
    if (DEBUG_WRITE_LOG):
        if (logPath == None):
            logPath = DEBUG_LOG_PATH
        f = open(logPath, "a")
        f.write(string + "\n")
        f.close()
    if (DEBUG_PRINT_LOG):
        print(string)
    return

globalRuntimeAnalysis = {}
# Support runtime tracking and store in dictionary of at most three level
def runtime(key1, key2=None, key3=None):
    def deco(func):
        @functools.wraps(func)
        def fun(*args, **kwargs):
            t = datetime.datetime.now()
            result = func(*args, **kwargs)
            dt = (datetime.datetime.now() - t).total_seconds()
            global globalRuntimeAnalysis
            if (key1 != None and key2 != None and key3 != None):
                # Store in globalRuntimeAnalysis[key1][key2][key3]
                if (key1 not in globalRuntimeAnalysis):
                    globalRuntimeAnalysis[key1] = {}
                if (key2 not in globalRuntimeAnalysis[key1]):
                    globalRuntimeAnalysis[key1][key2] = {}
                if (key3 not in globalRuntimeAnalysis[key1][key2]):
                    globalRuntimeAnalysis[key1][key2][key3] = [dt, 1]
                else:
                    globalRuntimeAnalysis[key1][key2][key3][0] += dt
                    globalRuntimeAnalysis[key1][key2][key3][1] += 1
            elif (key1 != None and key2 != None and key3 == None):
                # Store in globalRuntimeAnalysis[key1][key2]
                if (key1 not in globalRuntimeAnalysis):
                    globalRuntimeAnalysis[key1] = {}
                if (key2 not in globalRuntimeAnalysis[key1]):
                    globalRuntimeAnalysis[key1][key2] = [dt, 1]
                else:
                    globalRuntimeAnalysis[key1][key2][0] += dt
                    globalRuntimeAnalysis[key1][key2][1] += 1
            elif (key1 != None and key2 == None and key3 == None):
                # Store in globalRuntimeAnalysis[key1]
                if (key1 not in globalRuntimeAnalysis):
                    globalRuntimeAnalysis[key1] = [dt, 1]
                else:
                    globalRuntimeAnalysis[key1][0] += dt
                    globalRuntimeAnalysis[key1][1] += 1
            return result
        return fun
    return deco