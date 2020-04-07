from itertools import combinations

import networkx as nx
import numpy as np

from modsim.params import cage_width


def docking_array_to_graph(arr, n):
    """
    Decode the array and convert it into a graph
    :param arr: docking array
    :param n: 
    :return: 
    """
    # create graph
    G = nx.Graph()
    G.add_nodes_from(range(n))

    # possible connections for n robots
    pairs = list(combinations(range(n), 2))

    # Create graph edges based on the component arrary
    for (i, j), dc in zip(np.array(pairs)[arr > 0], arr[arr > 0]):
        # relative locations
        if dc == 1:  # up 1
            mov = [cage_width, 0.]
        elif dc == 2:  # right 2
            mov = [0., -cage_width]
        elif dc == 3:  # down 3
            mov = [-cage_width, 0.]
        elif dc == 4:  # left 4
            mov = [0, cage_width]
        G.add_edge(i, j, dock=mov)

    return G


def components_locations(G, n):
    """
    Convert graph to locations.    
    :param G: 
    :param n: 
    :return: location of each node in its component. 
    """
    zero2d = np.array([0., 0.])
    # location for each node in the local frame
    loc = [zero2d for i in range(n)]

    # For each subgraph, compute its structure
    for C in nx.connected_components(G):
        # Minimum id in component
        i = min(C)
        # remove minimum from i
        C.remove(i)

        def move_neighbors(i, G, C, loc):
            for j in G.neighbors(i):
                if j in C:
                    mov = G[i][j]['dock']

                    if j < i:
                        mov = -np.array(mov)

                    loc[j] = loc[i] + mov

                    C.remove(j)

                    # Expand recursively
                    move_neighbors(j, G, C, loc)
            return

        move_neighbors(i, G, C, loc)

    return np.array(loc)
