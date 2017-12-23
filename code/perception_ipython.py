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
	# Uncomment the below to limit the lookahead
    #lookahead = 4 * 10 # 4 meters * 10 pixels/meter
    #mask[0:(grid_img.shape[0] - bottom_offset - lookahead),:] = 0
    #warped = np.absolute(np.float32(warped - 1)) * mask

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
