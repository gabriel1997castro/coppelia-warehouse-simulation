import cv2
import numpy as np
import time
from skimage.metrics import structural_similarity as ssim
import astarsearch
import traversal

import os
dirname = os.path.dirname(__file__)

def main(image_filename):
    filename = os.path.join(dirname, image_filename)

    occupied_grids = []
    planned_path = {}

    #load the image
    image = cv2.imread(filename)
    (winW, winH) = (60, 60)

    obstacles = []
    index = [1, 1]

    #create a blank, initialized a matrix of 0s
    blank_image = np.zeros((60, 60, 3), np.uint8)

    #create an array of 100 blank images
    list_images = [[blank_image for i in range(10)]]
    maze = [[0 for i in range(10) for i in range(10)]]

    #traversal time! detect non-empty squares
    for(x, y, window) in traversal.sliding_window(image, stepSize=60, windowSize=(winW, winH)):
        # if the window does't meet our desired window size, ignore it
        if window.shape[0] != winH or window.shape[1] != winW:
            continue
        #print index, image is our iterator
        clone = image.copy()
        #format the suqare openCV
        cv2.rectangle(clone, (x, y), (x + winW, y + winH), (0, 255, 0), 2)
        crop_img = image[x:x + winW, y:y +winH]
        list_images[index[0]-1][index[1]-1] = crop_img.copy()

        #print the occupied grids
        average_color_per_row = np.average(crop_img, axis=0)
        average_color = np.average(average_color_per_row, axis=0)
        average_color = np.uint8(average_color)

        #iterate through the color matrix
        if(any(i<=240 for i in average_color)):
            print(index)
            maze[index[1]-1][index[0]-1] = 1
            occupied_grids.append(tuple(index))

        if(any(i <= 20 for i in average_color)):
            obstacles.append(tuple(index))
        
        cv2.imshow("Window", clone)
        cv2.waitKey(1)
        time.sleep(0.025)

        #Iterate
        index[1] = index[1] + 1						
        if (index[1]>10):
            index[0] = index[0] + 1
            index[1] = 1


    #Now it's time to perform the shortest path search
    #get the list of objects
    list_colored_grids = [n for n in occupied_grids if n not in obstacles]



    for startimage in list_colored_grids:
        key_startimage = startimage
        img1 = list_images[startimage[0]-1][startimage[1] -1]
        for grid in [n for n in list_colored_grids if n != startimage]:
        #next image
            img = list_images[grid[0]-1][grid[1]-1]
            #convert to grayscale
            image = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            image2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # compare structural similarity
            s = ssim(image, image2)
            #if they are similar, we'll perform a star search
            if s> 0.9:
                #perform a star - search a path - shortest path
                result = astarsearch.astar(maze,(startimage[0]-1,startimage[1]-1),(grid[0]-1,grid[1]-1))
                #print the result
                list2 = []
                for t in result:
                    x, y = t[0], t[1]
                    list2.append(tuple(x+1, y+1))
                    result = list(list2[1:-1])

    for obj in list_colored_grids:
        if not(planned_path.has_key(obj)):
            planned_path[obj] = list(["NO MATCH", [], 0])
    
    return occupied_grids, planned_path

if __name__ == '__main__':
    image_filename = 'assets/test_image1.jpg'
    main(image_filename)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
