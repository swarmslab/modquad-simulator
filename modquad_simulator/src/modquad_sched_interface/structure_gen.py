import numpy as np
#from scheduler.scheduler.modset import modset
from compiled_scheduler.modset import modset

def square(n):
    num_mod = n * n
    struc = np.ones((n,n))
    return modset(num_mod, struc)

def rect(length, width):
    return modset(length*width, np.ones((length, width)))

def zero(height, base):
    struc = np.zeros((height, base))
    struc[0, :] = 1
    struc[:, 0] = 1
    struc[height-1, :] = 1
    struc[:, base-1] = 1
    num_mod = len(np.nonzero(struc)[0])
    return modset(num_mod, struc)

def plus(height, base):
    struc = np.zeros((height, base))
    struc[int(height/2), :] = 1
    struc[:, int(base/2)] = 1
    num_mod = len(np.nonzero(struc)[0])
    return modset(num_mod, struc)

def side_ushape(height, base):
    struc = np.zeros((base, height))
    struc[0, :] = 1
    struc[:, 0] = 1
    struc[:, -1]=1
    num_mod = len(np.nonzero(struc)[0])
    return modset(num_mod, struc)

def fwd_ushape(height, base):
    struc = np.zeros((base, height))
    struc[0, :] = 1
    struc[:, 0] = 1
    struc[:, -1]=1
    struc = np.transpose(struc)
    num_mod = len(np.nonzero(struc)[0])
    return modset(num_mod, struc)

def airplane(width, longbar, sidebar):
    """
    Generates structure that looks like
        o
    o   o   o
    o o o o o
    o   o   o
        o
    """
    struc = np.zeros((longbar, width))
    center = int(longbar / 2)
    struc[:, center] = 1
    start_sidebar = longbar - sidebar - 1
    struc[start_sidebar:start_sidebar + sidebar,  0] = 1
    struc[start_sidebar:start_sidebar + sidebar, -1] = 1
    struc[int(width/2), :] = 1
    num_mod = len(np.nonzero(struc)[0])
    return modset(num_mod, struc)
