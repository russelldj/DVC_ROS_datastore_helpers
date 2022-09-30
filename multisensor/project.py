import numpy as np
import pdb


def filter_points_to_image_extent(points, img_size, offset=0.5, return_indices=True):
    """Return points which are within an image extents
    
    The returned points will be such that rounding them will allow valid indexing into the image.
    Therefore, the minimum valid points is -0.5 and the maximum is axis_size - 0.5.

    inputs:
        points: np.array
            (2, n). The projected points in image coordinates
        img_size: tuple(int):
            (width, height) in pixels
        offset: float
            How to account for subpixel indexing. 0.5 return indices that are valid after rounding. 
            0.0 will return indices valid after truncation.
        return_indices: bool
            Whether to return the indices that were used
        
    returns:
        np.array:
            Valid points after filtering. (2, n).
    """
    # Account for differences in how matrices and arrays deal with indexing a single dimension
    points = np.asarray(points)

    img_size_x, img_size_y = img_size
    inside = np.logical_and.reduce(
        (
            points[0] > (-offset),
            points[1] > (-offset),
            points[0] < (img_size_x - offset),
            points[1] < (img_size_y - offset),
        )
    )
    points_inside = points[:, inside]

    if return_indices:
        return points_inside, inside

    return points_inside


def sample_points(img, img_points):
    """Sample the values from an image based on coordinates

    inputs:
        img: np.array
            (h, w, 3) or (h, w). The image to sample from
        img_points: np.array
            (2, n). float or int. Points to sample from. Assumed to be (x, y).T

    returns:
        np.array
        Sampled points concatenated vertically
    """
    if isinstance(img_points, np.matrix):
        # Avoid weird behavior
        img_points = np.asarray(img_points)

    # Force non-int points to be ints
    if issubclass(img_points.dtype.type, np.floating):
        img_points = np.round(img_points).astype(np.uint16)

    sampled_values = img[img_points[1], img_points[0]]
    return sampled_values


def texture_lidar(lidar, image, intrinsics, extrinsics=None):
    colors = np.ones((lidar.shape[0], 3)) * 128
    if image is None:
        return colors, np.zeros(colors.shape[0], dtype=bool)

    if extrinsics is not None:
        lidar_homog = np.concatenate((lidar, np.ones((lidar.shape[0], 1))), axis=1)
        # Perform the matrix multiplication to get the lidar points into the local frame
        lidar_proj = np.dot(extrinsics, lidar_homog.T)
        lidar_transformed = lidar_proj[:3]
    else:
        # Keep the lidar frame the same as it was initially
        lidar_transformed = lidar.T

    # Note that lidar_transformed is (3, n) to facilitate easier multiplication
    in_front_of_camera = lidar_transformed[2] > 0
    # Take only points which are in front of the camera
    lidar_transformed_filtered = lidar_transformed[:, in_front_of_camera]
    # Project each point into the image
    projections_homog = np.dot(intrinsics, lidar_transformed_filtered)
    projections_inhomog = projections_homog[:2] / projections_homog[2]

    img_height, img_width, _ = image.shape

    # TODO consider using the shape from the current image
    image_points, within_bounds = filter_points_to_image_extent(
        projections_inhomog, (img_width, img_height)
    )
    projections_inhomog = np.asarray(projections_homog)

    sampled_colors = sample_points(image, image_points)
    valid_inds = in_front_of_camera
    # Set all true values to whether or not they're in bounds
    valid_inds[valid_inds] = within_bounds
    colors[valid_inds] = sampled_colors
    return colors, valid_inds
