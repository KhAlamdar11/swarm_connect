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

def hungarian_reassignment(current_positions, desired_positions):
    n = current_positions.shape[0]  # Number of robots/positions
    
    # Initialize the cost matrix with zeros
    cost_matrix = np.zeros((n, n))
    
    # Calculate the cost (Euclidean distance) for each robot to each position
    for i in range(n):
        for j in range(n):
            cost_matrix[i, j] = np.linalg.norm(current_positions[i] - desired_positions[j])
    
    # Solve the assignment problem
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    
    # Reorder desired_positions based on the assignment
    reordered_desired_positions = desired_positions[col_ind]
    
    # Return the reordered desired_positions matrix
    return reordered_desired_positions

# Example usage:
# positions = np.array([[0, 0], [1, 1], [2, 2], [5, 5]])
# communication_distance = 3
# edges = get_edges(positions, communication_distance)
# print(edges)
