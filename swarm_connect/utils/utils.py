import numpy as np
from scipy.optimize import linear_sum_assignment

def get_edges(positions, communication_distance):
    """
    Given an array of positions and a communication distance, 
    returns a list of edges where each edge is a pair of indices 
    representing points that are less than the communication distance apart.
    
    :param positions: An (n x 2) numpy array of [x, y] positions.
    :param communication_distance: The maximum distance for an edge to exist.
    :return: A list of tuples where each tuple contains the indices of the points 
             that form an edge.
    """
    edges = []
    # Calculate the distance matrix
    distance_matrix = np.sqrt(np.sum((positions[:, np.newaxis] - positions[np.newaxis, :]) ** 2, axis=-1))

    # Iterate over each pair of positions
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):  # Only check each pair once
            if distance_matrix[i, j] < communication_distance:
                edges.append((i, j))
    return edges