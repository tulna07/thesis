"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""

from __future__ import print_function
import cv2 as cv
import csv

class World:
    def __init__(self) -> None:
        pass

    def thresh_callback(self, threshold, src_gray, world_name):
        # Detect edges using Canny
        canny_output = cv.Canny(src_gray, threshold, threshold * 2)
        # Find contours
        contours, hierarchy = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        file_name = world_name + ".csv"
        data_header = ["x", "y"]
        f = open(file_name, 'w', newline='')
        writer = csv.writer(f, delimiter=",")
        # only keep parents in list, remove its child(ren)
        for i in range (len(contours)):
            
            if hierarchy[0][i][3] == -1:   # this contour is parent
                part = contours[i]
                writer.writerow(data_header)
                for pt in part:
                    writer.writerow([pt[0][0], pt[0][1]])
        f.close()


    # Load source image
    def read_map_from_world(self, world_name):
        src = cv.imread(world_name)
        if src is None:
            print('Could not open or find the image:', world_name)
            exit(0)

        src = cv.flip(src, 0)

        # Convert image to gray and blur it
        src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        src_gray = cv.blur(src_gray, (3, 3))

        thresh = 100  # initial threshold
        self.thresh_callback(thresh, src_gray, world_name)


    def display(self, plt, mpimg, world_name):
        img = mpimg.imread(world_name)
        plt.imshow(img)
