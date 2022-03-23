import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

def get_best_point(ori_bin_img):
    """Finding the best goal

    Args:
        ori_bin_img (np.array)

    Returns:
        [pixel_x, pixel_y] on the image
        
    TO DO:
    1. Goal Point should be mean value of one area
    2. Transfer the pixel position into world position
    """
    
    costmap=cv2.distanceTransform(ori_bin_img,cv2.DIST_L2,0) # Generate the costmap
    costmap_roi = costmap[119:210,:] # Locate the range of interest(ROI)
    min_value,max_value,minloc,maxloc = cv2.minMaxLoc(costmap) # Find the best goal point
        
    return costmap, maxloc


if __name__ =='__main__':
    # Load testing images
    image_filename = "Obstacle-avoidance-for-smartcar-with-monocular-vision-/images"

	# Sort the file name in correct order
    files= os.listdir(image_filename) # Obtain all images
    files = sorted(files)
    
    for file in files:
        '''
        Load the Image and Subsample the Image
        
        input: image(240*520)
        output: image(60*130) with gaussian blur to filter the noise
        '''
        # Load the image from file
        image = cv2.imread(image_filename+"/"+file)
        image = np.rot90(image)
        
        #size = (image.shape[0]/4, image.shape[1]/4)
        size = (130,60)
        image = cv2.resize(image,size)
        
        image = cv2.GaussianBlur(image, (5,5),1)

        '''
        Extract the grass(green) area
        '''
        # Transfer the image into HSV channel
        img_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
        
        # Obtain the green pixels 
        low_hsv = np.array([20, 43, 46])
        high_hsv = np.array([60, 255, 255])
        mask = cv2.inRange(img_HSV, low_hsv, high_hsv)
        
        
        # YUV = cv2.cvtColor(image, cv2.COLOR_BGR2YUV) 
        # Filtered = np.zeros([YUV.shape[0], YUV.shape[1]]);
        # for y in range(YUV.shape[0]):
        #     for x in range(YUV.shape[1]):
        #         if(YUV[y,x,0] >= 50 and YUV[y,x,0] <= 200 and \
        #             YUV[y,x,1] >= 0 and YUV[y,x,1] <= 130 and \
        #             YUV[y,x,2] >= 0 and YUV[y,x,2] <= 220):
        #                 Filtered[y,x] = 1;
        # Filtered = cv2.normalize(Filtered, None, 255,0, cv2.NORM_MINMAX, cv2.CV_8UC1)
        '''
        Calculate the Goal based on the bin image
        '''
        costmap, maxloc = get_best_point(mask)
        
        '''
        Visualize the Depth Image
        '''
        dist1 = cv2.convertScaleAbs(costmap)
        dist2 = cv2.normalize(dist1, None, 255,0, cv2.NORM_MINMAX, cv2.CV_8UC1)
        heat_img = cv2.applyColorMap(dist2, cv2.COLORMAP_JET)
        
        cv2.imshow('heatmap',heat_img)
        cv2.imshow('filter',mask)
        cv2.imshow('origin',image)
        cv2.waitKey(200)
        #print(maxloc)