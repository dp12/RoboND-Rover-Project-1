import numpy as np
import cv2

class Parm:
    cutoff = 1
class Cell:
    UNKNOWN, FREE, OBSTACLE, ROCK = range(4)

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh_navigable(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_thresh_obs(img, rgb_threshold=(160, 160, 160)):
    color_select = np.zeros_like(img[:,:,0])
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    color_select[below_thresh] = 1
    return color_select

def color_thresh_rock(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # target is 46-53 --> 92 - 106 90-100 works well
    low_thresh = np.array([20,50,50])
    high_thresh = np.array([30,255,255])
    color_select = cv2.inRange(hsv, low_thresh, high_thresh)
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
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

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    return warped, mask


def perception_step_ipython(img):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in img
    # 1) Define source and destination points for perspective transform
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                              [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
    ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(img, source, destination)
    lookahead = 4 * 10 # 4 meters * 10 pixels/meter
    mask[0:(grid_img.shape[0] - bottom_offset - lookahead),:] = 0
    warped = np.absolute(np.float32(warped - 1)) * mask

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed_rocks = color_thresh_rock(warped)
    threshed_navigable = color_thresh_navigable(warped)
    threshed_obs = np.absolute(np.float32(threshed_navigable - 1)) * mask
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    # Rover.vision_image[:,:,0] = threshed_obs*255
    # Rover.vision_image[:,:,1] = threshed_rock*255
    # Rover.vision_image[:,:,2] = threshed_navigable*255

    # 5) Convert map image pixel values to rover-centric coords
    obs_x_rover, obs_y_rover = rover_coords(threshed_obs)
    rocks_x_rover, rocks_y_rover = rover_coords(threshed_rocks)
    navigable_x_rover, navigable_y_rover = rover_coords(threshed_navigable)
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 10
    obs_x_world, obs_y_world = pix_to_world(obs_x_rover, obs_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], world_size, scale)
    rocks_x_world, rocks_y_world = pix_to_world(rocks_x_rover, rocks_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], world_size, scale)
    navigable_x_world, navigable_y_world = pix_to_world(navigable_x_rover, navigable_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # Rover.worldmap[obs_y_world, obs_x_world] = (255, 0, 0)
    # Rover.worldmap[navigable_y_world, navigable_x_world] = (0, 0, 255)
    Rover.worldmap[obs_y_world, obs_x_world, 0] = 255
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] = 255
    Rover.worldmap[rocks_y_world, rocks_x_world] = (255, 255, 255)

    nav_pix = Rover.worldmap[:,:,2] > 0
    Rover.worldmap[nav_pix, 0] = 0

    # Rover.worldmap[rocks_y_world, rocks_x_world] = (0, 255, 0)
    # nav_pix = Rover.worldmap[:,:,2] > 0
    # Rover.worldmap[nav_pix, 0] = 0
    # Rover.worldmap[navigable_y_world, navigable_x_world] = (0, 0, 255)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    # navigable_dist, navigable_angles = to_polar_coords(navigable_x_rover, navigable_y_rover)
    # Rover.nav_dists = navigable_dist
    # Rover.nav_angles = navigable_angles

    return img

# cmd /k "activate robond & python drive_rover.py"
# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                              [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
    ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)
    lookahead = 4 * 10 # 4 meters * 10 pixels/meter
    mask[0:(Rover.img.shape[0] - bottom_offset - lookahead),:] = 0

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # threshed_obs = color_thresh_obs(warped)
    threshed_rocks = color_thresh_rock(warped)
    threshed_navigable = color_thresh_navigable(warped)
    threshed_obs = np.absolute(np.float32(threshed_navigable - 1)) * mask
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    # print(threshed_obs)
    Rover.vision_image[:,:,0] = threshed_obs*255
    Rover.vision_image[:,:,1] = threshed_rocks*255
    Rover.vision_image[:,:,2] = threshed_navigable*255

    # 5) Convert map image pixel values to rover-centric coords
    obs_x_rover, obs_y_rover = rover_coords(threshed_obs)
    rocks_x_rover, rocks_y_rover = rover_coords(threshed_rocks)
    navigable_x_rover, navigable_y_rover = rover_coords(threshed_navigable)
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    obs_x_world, obs_y_world = pix_to_world(obs_x_rover, obs_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    # rocks_x_world, rocks_y_world = pix_to_world(rocks_x_rover, rocks_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    navigable_x_world, navigable_y_world = pix_to_world(navigable_x_rover, navigable_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.bitmap[navigable_y_world, navigable_x_world] = Cell.FREE
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] = 255
    Rover.worldmap[obs_y_world, obs_x_world, 0] = 255
    Rover.bitmap[obs_y_world, obs_x_world] = Cell.OBSTACLE
    nav_pix = Rover.worldmap[:,:,2] > 0
    Rover.worldmap[nav_pix, 0] = 0

    rock_map = color_thresh_rock(warped)
    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
        Rover.worldmap[rock_y_world, rock_x_world, :] = 255
        Rover.bitmap[rock_y_world, rock_x_world] = Cell.ROCK

        rock_dist, rock_angles = to_polar_coords(rocks_x_rover, rocks_y_rover)
        Rover.nav_rock_dists = rock_dist
        Rover.nav_rock_angles = rock_angles

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    navigable_dist, navigable_angles = to_polar_coords(navigable_x_rover, navigable_y_rover)
    Rover.nav_dists = navigable_dist
    Rover.nav_angles = navigable_angles


    if rock_map.any():
        if np.mean(Rover.nav_rock_dists) < 10:
            Rover.mode = 'goto_rock'
            print("Saw rock")
    else:
        Rover.mode = 'forward'
    goal_dist, goal_angle = overmind()

    return Rover

def straight_walker(Rover, fstep):
    end_xpos, end_ypos = Rover.pos[0], Rover.pos[1]
    xpos, ypos = Rover.pos[0], Rover.pos[1]
    num_fsteps = 0
    while True:
        xpos = xpos + fstep * np.sin(Rover.yaw)
        ypos = ypos + fstep * np.cos(Rover.yaw)
        xcell, ycell = int(xpos), int(ypos) #round down to get cell index
        if Rover.bitmap(ycell, xcell) == Cell.FREE:
            end_xpos, end_ypos = xpos, ypos
            num_fsteps += 1
        else:
            break;
    return end_xpos, end_ypos, num_fsteps

def horizontal_walker(Rover, fwd_xpos, fwd_ypos, fwd_dist, hstep, side):
    # Get by pythagorean theorem
    hdist = 0
    xpos, ypos = fwd_xpos, fwd_ypos
    while True:
        hdist += hstep
        hstep_angle = np.atan2(hdist / fwd_dist)
        if side == 'left':
            target_world_angle = Rover.yaw - hstep_angle
        elif side == 'right':
            target_world_angle = Rover.yaw + hstep_angle
        else:
            raise ValueError
        r = np.sqrt((fwd_dist)**2 + hdist**2)
        xpos = r * np.sin(target_world_angle)
        ypos = r * np.cos(target_world_angle)
        xcell, ycell = int(xpos), int(ypos) #round down to get cell index
        if Rover.bitmap(ycell, xcell) == Cell.OBSTACLE:
            # or if max hdist exceeded
            break;
    return xpos, ypos

def midpoint(p1_x, p1_y, p2_x, p2_y):
    mid_x = (p2_x - p1_x) / 2 + p1_x
    mid_y = (p2_y - p1_y) / 2 + p1_y
    return mid_x, mid_y

@static_vars(last_message="")
def overmind(Rover):
    # Step 1: Walk straight ahead until non-FREE cell hit
    fstep = np.sqrt(2)
    fwd_xpos, fwd_ypos, num_fsteps = straight_walker(Rover.pos[0], Rover.pos[1], Rover.yaw, fstep)
    '''
    xpos = Rover.pos[0] + fstep * np.sin(Rover.yaw)
    ypos = Rover.pos[1] + fstep * np.cos(Rover.yaw)
    xcell, ycell = int(xpos), int(ypos) #round down to get cell index
    num_fsteps = 1
    while True:
        if Rover.bitmap(ycell, xcell) == Cell.FREE:
            last_xpos, last_ypos = xpos, ypos
            xpos = xpos + fstep * np.sin(Rover.yaw)
            ypos = ypos + fstep * np.cos(Rover.yaw)
            xcell, ycell = int(xpos), int(ypos) #round down to get cell index
            num_fsteps += 1
        else:
            break;
    '''
    if (num_fsteps >= Parm.cutoff):
        message = "Straight walker is %d %d".format(xcell, ycell)
        if last_message != message:
            print message
            overmind.last_message = message

        # Step 2: Walk in horizontal line perpendicular to Rover heading and
        # compute the midpoint between obstacles, if any
        fwd_dist = fstep * num_fsteps
        hstep = fstep / 2 # walk for half diagonal
        left_xpos, left_ypos = horizontal_walker(fwd_xpos, fwd_ypos, hstep, 'left')
        right_xpos, right_ypos = horizontal_walker(fwd_xpos, fwd_ypos, hstep, 'right')
        target_xpos, target_ypos = midpoint(left_xpos, left_ypos, right_xpos, right_ypos)
    else:
        # Closest pathline is less than cutoff, do a turn-in-place cw
        Rover.mode = 'tip'

if __name__=="__main__":
   main()
