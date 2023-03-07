import os
import numpy as np
import open3d as o3d

def iterative_closet_point(source, target):
    """
    This code performance the iterative closet point algorithm
    :param source: source point cloud
    :param target: target point cloud
    :return: the transformed source point cloud
    """
    # Initialize the result point cloud
    result = source

    # Initialize the threshold
    threshold = 0.02

    # Initialize the maximum iteration
    max_iteration = 30

    # Initialize the transformation matrix
    transformation = np.identity(4)

    # Initialize the distance
    distance = 1e10

    # Initialize the iteration
    iteration = 0

    # Iterate until the distance is smaller than the threshold or the iteration is larger than the maximum iteration
    while distance > threshold and iteration < max_iteration:
        # Find the closest point
        closest_point, distance, _ = result.kdtree.search_knn_vector_3d(target.points, 1)

        # Initialize the transformation matrix
        transformation = np.identity(4)

        # Compute the transformation matrix
        transformation[0:3, 0:3] = result.compute_point_cloud_moment().dot(target.compute_point_cloud_moment()).T
        transformation[0:3, 3] = target.compute_mean() - result.compute_mean().dot(transformation[0:3, 0:3])

        # Transform the point cloud
        result.transform(transformation)

        # Update the iteration
        iteration += 1

    return result


def main():
    point_source = o3d.io.read_point_cloud("data/point_source.ply")
    point_target = o3d.io.read_point_cloud("data/point_target.ply")

    point_result = iterative_closet_point(point_source, point_target)

if __name__ == "__main__":
    main()