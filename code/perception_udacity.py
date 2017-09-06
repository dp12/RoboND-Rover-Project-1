# Define a function to pass stored images to
# reading rover position and yaw angle from csv file
# This function will be used by moviepy to create an output video
def process_image(img):
    # Example of how to use the Databucket() object defined above
    # to print the current x, y and yaw values 
    # print(data.xpos[data.count], data.ypos[data.count], data.yaw[data.count])

    # TODO: 
    # 1) Define source and destination points for perspective transform
    source = np.float32([[14, 140], [301, 140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                              [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
    ])

    # 2) Apply perspective transform
    warped, mask = perspect_transform(img, source, destination)


    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # threshed_rocks = color_thresh_rock(warped)
    # threshed_navigable = color_thresh_navigable(warped)
    # threshed_obs = np.absolute(np.float32(threshed_navigable - 1)) * mask
    threshed = color_thresh(warped)
    obs_map = np.absolute(np.float32(threshed - 1)) * mask

    # 4) Convert thresholded image pixel values to rover-centric coords
    # obs_x_rover, obs_y_rover = rover_coords(threshed_obs)
    # rocks_x_rover, rocks_y_rover = rover_coords(threshed_rocks)
    # navigable_x_rover, navigable_y_rover = rover_coords(threshed_navigable)
    xpix, ypix = rover_coords(threshed)
    obs_x_pix, obs_y_pix = rover_coords(obs_map)

    # 5) Convert rover-centric pixel values to world coords
    world_size = data.worldmap.shape[0]
    scale = 2 * dst_size
    xpos = data.xpos[data.count]
    ypos = data.ypos[data.count]
    yaw = data.yaw[data.count]

    x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    obs_x_world, obs_y_world = pix_to_world(obs_x_pix, obs_y_pix, xpos, ypos, yaw, world_size, scale)
    # obs_x_world, obs_y_world = pix_to_world(obs_x_rover, obs_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], world_size, scale)
    # rocks_x_world, rocks_y_world = pix_to_world(rocks_x_rover, rocks_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], world_size, scale)
    # navigable_x_world, navigable_y_world = pix_to_world(navigable_x_rover, navigable_y_rover, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], world_size, scale)

    # 6) Update worldmap (to be displayed on right side of screen)
        # Example: data.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          data.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          data.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    #data.worldmap[obs_y_world, obs_x_world] = (255, 0, 0)
    #data.worldmap[navigable_y_world, navigable_x_world] = (0, 0, 255)
    #data.worldmap[rocks_y_world, rocks_x_world] = (255, 255, 255)
    # data.worldmap[obs_y_world, obs_x_world, 0] = 255
    # data.worldmap[navigable_y_world, navigable_x_world, 2] = 255
    # data.worldmap[rocks_y_world, rocks_x_world] = (255, 255, 255)
    data.worldmap[y_world, x_world, 2] = 255
    data.worldmap[obs_y_world, obs_x_world, 0] = 255
    nav_pix = data.worldmap[:,:,2] > 0

    data.worldmap[nav_pix, 0] = 0

    rock_map = color_thresh_rock(warped)
    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world_size, scale)
        data.worldmap[rock_y_world, rock_x_world, :] = 255

    # 7) Make a mosaic image, below is some example code
        # First create a blank image (can be whatever shape you like)
    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
        # Next you can populate regions of the image with various output
        # Here I'm putting the original image in the upper left hand corner
    output_image[0:img.shape[0], 0:img.shape[1]] = img

        # Let's create more images to add to the mosaic, first a warped image
    warped, mask = perspect_transform(img, source, destination)
        # Add the warped image in the upper right hand corner
    output_image[0:img.shape[0], img.shape[1]:] = warped

        # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
        # Flip map overlay so y-axis points upward and add to output_image 
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)


        # Then putting some text over the image
    cv2.putText(output_image,"Populate this image with your analyses to make a video!", (20, 20), 
                cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    if data.count < len(data.images) - 1:
        data.count += 1 # Keep track of the index in the Databucket()
    
    return output_image
