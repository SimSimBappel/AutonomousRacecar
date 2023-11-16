import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.distance import cdist
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def load_points_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        coordinates = [list(map(float, line.strip().split(','))) for line in lines]
    return np.array(coordinates)


def icp_registration(source, target, threshold=0.05, max_iterations=50):
    # Initialize transformation matrix
    transformation_matrix = np.eye(3)

    # Define the objective function for optimization
    def objective_function(params, source, target):
        transformation_matrix = np.eye(3)
        transformation_matrix[:2, :2] = params.reshape((2, 2))
        transformation_matrix[:2, 2] = params[2:]
        
        transformed_source = np.dot(np.column_stack((source, np.ones(len(source)))), transformation_matrix.T)[:, :2]
        return np.sum(cdist(transformed_source, target, metric='euclidean').min(axis=1))

    # Perform ICP registration using optimization
    result = minimize(objective_function, transformation_matrix[:2, :2].flatten(),
                      args=(source, target), method='L-BFGS-B')

    # Extract the optimized transformation matrix
    transformation_matrix[:2, :2] = result.x.reshape((2, 2))
    transformation_matrix[:2, 2] = result.x[2:]

    return transformation_matrix

def plot_map_lidar_transform(map_points, lidar_points, transformation_matrix):
    # Transform lidar points using the calculated transformation
    transformed_lidar = np.dot(np.column_stack((lidar_points, np.ones(len(lidar_points)))), transformation_matrix.T)[:, :2]

    # Plot map points, lidar points, and the transformation
    plt.figure(figsize=(10, 8))

    plt.scatter(map_points[:, 0], map_points[:, 1], c='blue', label='Map Points')
    plt.scatter(lidar_points[:, 0], lidar_points[:, 1], c='green', label='LiDAR Readings')
    plt.scatter(transformed_lidar[:, 0], transformed_lidar[:, 1], c='red', label='Transformed LiDAR Readings')

    # Plot transformation arrow
    plt.quiver(0, 0, transformation_matrix[0, 2], transformation_matrix[1, 2],
               angles='xy', scale_units='xy', scale=1, color='orange', label='Transform Vector')

    plt.title('Map, LiDAR Readings, and Transformed LiDAR Readings')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    # Load map and LiDAR readings
    map_path = '/home/simon/autonomousRacecar/src/pyslam/webotskeypointsmap.txt'
    lidar_path = '/home/simon/autonomousRacecar/src/pyslam/webotskeypointsobserved.txt'

    map_points = load_points_from_file(map_path)
    lidar_points = load_points_from_file(lidar_path)

    # Perform ICP registration
    transformation_matrix = icp_registration(lidar_points, map_points)

    # Extract translation and rotation
    translation = transformation_matrix[:2, 2]
    rotation_matrix = transformation_matrix[:2, :2]
    rotation = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])  # Assuming 2D rotation

    # Print results
    print("Estimated Transformation:")
    print("Translation:", translation)
    print("Rotation (angle in radians):", rotation)

     # Plot the map, LiDAR readings, and transformed LiDAR readings
    plot_map_lidar_transform(map_points, lidar_points, transformation_matrix)

if __name__ == "__main__":
    main()
