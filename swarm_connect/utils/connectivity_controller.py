import numpy as np
from scipy.spatial.distance import pdist, squareform
from scipy.linalg import eig
from numpy.linalg import inv

class ConnectivityController:

    def __init__(self,params):
        self.params = params
        self.fiedler_value = None
        self.fiedler_vector = None


    def __call__(self, position, battery):

        nv = position.shape[0]
        
        positionx, positiony, ac, eigV = self.set_controlData(position)
        
        self.fiedler_value = ac
        self.fiedler_vector = eigV

        battery = np.concatenate((np.array([1.0, 1.0]), battery))

        # print(self.fiedler_value)

        A = self.initialize_matrixA(position)
        dotx = np.zeros(nv)
        doty = np.zeros(nv)

        for i in range(nv):
            for j in range(nv):
                k = 0
                if A[i, j] > 0:
                    if ac > self.params['epsilon']:
                        k = (-(1 / (self.params['sigma']**2)) * (self.csch(ac - self.params['epsilon'])**2)) * (A[i, j] * ((eigV[i] - eigV[j])**2))
                    else:
                        # k=0
                        k = -(1 / (self.params['sigma']**2)) * 100 * (A[i, j] * ((eigV[i] - eigV[j])**2))
                    
                    batt_gain = self.battery_gain(battery[j])

                    dotx[i] += k * (positionx[i] - positionx[j]) * 5*batt_gain
                    doty[i] += k * (positiony[i] - positiony[j]) * 5*batt_gain
        
        dotxy = np.column_stack([dotx, doty]) * self.params['gainConnectivity']
        
        dotxy = dotxy + self.calculate_repulsion_forces(position, self.params['repelThreshold'], self.params['gainRepel'])
        
        dotxy = self.clip(dotxy)

        return dotxy
    
    def battery_gain(self,b):
        # print(self.params['tau'])
        return np.exp((0.14 - b) / 0.2) + 1.2



    def calculate_repulsion_forces(self,positions, threshold, repulsion_strength):
        n = positions.shape[0]
        repulsion_vectors = np.zeros_like(positions)

        # Calculate pairwise distance and repulsion if within threshold
        for i in range(n):
            for j in range(i + 1, n):  # No need to check i with itself or redo pairs
                difference_vector = positions[i] - positions[j]
                distance = np.linalg.norm(difference_vector)
                if distance < threshold:
                    # Calculate repulsion vector (normalized to unit vector)
                    repulsion_vector = difference_vector / (distance ** 2)  # Inversely proportional to square of distance
                    repulsion_vectors[i] += repulsion_vector * repulsion_strength
                    repulsion_vectors[j] -= repulsion_vector * repulsion_strength  # Opposite direction for the other robot

        return repulsion_vectors


    def csch(self,x):
        return 1.0 / np.sinh(x)

    def degree(self,A):
        return np.diag(np.sum(A, axis=1))

    def algebraic_connectivity(self,A):
        D = self.degree(A)
        if np.all(np.diag(D) != 0):  # Check if there are no isolated vertices
            L = D - A
            if self.params['normalized']:
                D_inv_sqrt = inv(np.sqrt(D))
                L = D_inv_sqrt @ L @ D_inv_sqrt
            eValues, _ = eig(L)
            eValues = np.sort(eValues.real)  # Ensure eigenvalues are sorted in real number form
            ac = eValues[1]  # Second smallest eigenvalue
        else:
            ac = 0
        return ac

    def compute_eigVector(self, A):
        D = self.degree(A)
        L = D - A
        if self.params['normalized']:
            D_inv_sqrt = inv(np.sqrt(D))
            L = D_inv_sqrt @ L @ D_inv_sqrt
        eValues, eVectors = eig(L)
        Y = np.argsort(eValues.real)  # Sort eigenvalues and ensure they are in real number form
        v = eVectors[:, Y[1]]  # Eigenvector corresponding to the second smallest eigenvalue
        return v.real  # Return the real part of the eigenvector

    def initialize_matrixA(self, position):
        distance = squareform(pdist(position, 'euclidean'))
        matrix = np.zeros_like(distance)
        matrixW = np.exp(-(distance**2) / (2 * self.params['sigma']**2))
        r, c = np.where((self.params['range'] >= np.triu(distance, 0)) & (np.triu(distance, 0) != 0))
        for i in range(len(r)):
            weight = 1
            if not self.params['unweighted']:
                weight = matrixW[c[i], r[i]]
            matrix[r[i], c[i]] = weight
            matrix[c[i], r[i]] = weight

        matrix[0, :] /= 2
        matrix[:, 0] /= 2

        matrix[1, :] /= 2
        matrix[:, 1] /= 2

        return matrix

    def get_fiedler(self):
        return self.fiedler_value, self.fiedler_vector


    def set_controlData(self, position):
        A = self.initialize_matrixA(position)
        ac = self.algebraic_connectivity(A,)
        eigV = self.compute_eigVector(A)
        return position[:, 0], position[:, 1], np.array([ac]), np.array(eigV)


    def clip(self, velocities):
        magnitudes = np.linalg.norm(velocities, axis=1)
        with np.errstate(divide='ignore', invalid='ignore'):
            scale_factors = np.where(magnitudes > self.params['v_max'], self.params['v_max'] / magnitudes, 1)
        scaled_velocities = velocities * scale_factors[:, np.newaxis]
        return scaled_velocities
    
