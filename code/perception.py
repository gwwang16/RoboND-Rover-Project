import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only


def region_of_interest(img, vertices):
    """
    Applies an image mask.
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    # defining a blank mask to start with
    mask = np.zeros_like(img)
    # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    # returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def color_thresh(img, rgb_thresh=(120, 90, 80)):
    '''Select terrain area as binary image'''
    # Create an array of zeros same xy size as img, but single channel
    terrain_select = np.zeros_like(img[:, :, 0])
    # Using the bottom half to select sky area
    imshape = img.shape
    vertices = np.array([[(0, imshape[0]), (imshape[1], imshape[0]),
                          (imshape[1], imshape[0] * 0.3), (0, imshape[0] * 0.3)]], dtype=np.int32)
    masked_img = region_of_interest(img, vertices)
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (masked_img[:, :, 0] > rgb_thresh[0]) \
        & (masked_img[:, :, 1] > rgb_thresh[1]) \
        & (masked_img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    terrain_select[above_thresh] = 1

    # Return the binary image
    return terrain_select


def obstacle_thresh(img, terrain, rgb_thresh=([70, 70, 70], [160, 160, 160])):
    """
    Select the obstacle from not terrain area.
    The threshold is used for sky area selecting,
    The area, which are not sky & terrain, are considered as obstacle"""
    low_threshold = np.array(rgb_thresh[0], dtype="uint8")
    high_threshold = np.array(rgb_thresh[1], dtype="uint8")
    imshape = img.shape
    # Using the upper half to select sky area
    vertices = np.array([[(0, 0), (imshape[1], 0),
                          (imshape[1], imshape[0] * 0.4), (0, imshape[0] * 0.4)]], dtype=np.int32)
    masked_image = region_of_interest(img, vertices)
    sky_select = cv2.inRange(masked_image, low_threshold, high_threshold)

    # Get index of the sky and terrain
    sky_idx = sky_select.nonzero()
    terrain_idx = terrain.nonzero()

    # Create obstacle image, it is the reverse of sky and terrain
    obstacle_select = np.ones_like(img[:, :, 0])

    obstacle_select[sky_idx] = 0
    obstacle_select[terrain_idx] = 0

    return obstacle_select


def rock_thresh(img, rgb_thresh=([100, 100, 0], [190, 190, 50])):
    '''rock thresh function'''
    low_threshold = np.array(rgb_thresh[0], dtype="uint8")
    high_threshold = np.array(rgb_thresh[1], dtype="uint8")
    # Using mask fuction to select the rock
    rock_select = cv2.inRange(img, low_threshold, high_threshold)
    return rock_select


def rover_coords(binary_img):
    """Convert image to Rover-Centric Coordinates.
    Input: binary image
    Output: Rover centered coordinates image"""
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    """Convert x_pixel, y_pixel)
    to (distance, angle) radial coords in rover space"""
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions


def rotate_pix(xpix, ypix, yaw):
    """Apply a rotation to pixel positions using rotation matrix
    Input: xpix, ypix, yaw_rad
    Output: xpix_ratated, ypix_rotated"""
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    """Translate and scale pix to worldmap"""
    # Apply a scaling and a translation
    xpix_translated = xpos + (xpix_rot / scale)
    ypix_translated = ypos + (ypix_rot / scale)
    # Return the result
    return xpix_translated, ypix_translated


def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    """Combine the rotate_pix and translate_pix functions
    to perform world coordinates transform"""
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform


def perspect_transform(img, src, dst):
    """Perspective transform function
    to warp the camera image into top view"""
    M = cv2.getPerspectiveTransform(src, dst)
    # keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    """Perform perception steps to update Rover()"""
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = Rover.dst_size
    bottom_offset = Rover.bottom_offset
    source = np.float32([[14, 140], [300, 140], [200, 96], [119, 96]])
    destination = np.float32([[img.shape[1] / 2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1] / 2 + dst_size,
                               img.shape[0] - bottom_offset],
                              [img.shape[1] / 2 + dst_size, img.shape[0] -
                               2 * dst_size - bottom_offset],
                              [img.shape[1] / 2 - dst_size, img.shape[0] -
                               2 * dst_size - bottom_offset]])

    # 2) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain_threshold = (160, 160, 160)  # (130,120,100)
    obstacle_threshold = ([70, 70, 70], [160, 160, 160])
    rock_threshold = ([100, 100, 0], [200, 200, 50])

    terrain_select = color_thresh(img, terrain_threshold)
    rock_select = rock_thresh(img, rock_threshold)
    obstacle_select = obstacle_thresh(img, terrain_select, obstacle_threshold)

    # 3) Apply perspective transform
    terrain_select = perspect_transform(terrain_select, source, destination)
    rock_select = perspect_transform(rock_select, source, destination)
    obstacle_select = perspect_transform(obstacle_select, source, destination)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:, :, 0] = obstacle_select * 255
    Rover.vision_image[:, :, 1] = rock_select * 255
    Rover.vision_image[:, :, 2] = terrain_select * 255

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(terrain_select)
    xobstacle, yobstacle = rover_coords(obstacle_select)
    xrock, yrock = rover_coords(rock_select)

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    world_size = Rover.worldmap.shape[0]
    xpos, ypos = Rover.pos
    obstacle_x_world, obstacle_y_world = pix_to_world(xobstacle, yobstacle, xpos, ypos,
                                                      Rover.yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(xrock, yrock, xpos, ypos,
                                              Rover.yaw, world_size, scale)
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, xpos, ypos,
                                                        Rover.yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # mapping are valid when roll and pitch angles are near zero.
    roll_condition = [Rover.roll < 0.4, Rover.roll > 359.6]
    pitch_condition = [Rover.pitch < 0.4, Rover.pitch > 359.6]
    if any(roll_condition) and any(pitch_condition):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(xobstacle, yobstacle)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(xrock, yrock)

    return Rover
