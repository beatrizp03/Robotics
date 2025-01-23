import numpy as np
from scipy.spatial import procrustes

def file_to_nparray(filename):
    # Read the contents of the file
    data = []
    with open(filename, 'r') as f:
        for line in f:
            # Split the line by commas
            if line.strip():
                x, y = map(float, line.split(','))
                data.append([x, y])
    
    return np.array(data)

if __name__ == "__main__":
    ground_file = "EKF_ground.txt"
    ground_array = file_to_nparray(ground_file)

    estimated_file = "EKF_estimated.txt"
    estimated_array = file_to_nparray(estimated_file)
    
    # Uses procrustes algorithm to calculate the pointwise squared error after getting a rotation, 
    # transformation and scale factor that best aligns the two paths
    pts_ground, estimated_transformed, squared_error = procrustes(ground_array, estimated_array)
    print(squared_error)
