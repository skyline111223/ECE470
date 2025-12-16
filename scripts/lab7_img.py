#!/usr/bin/env python


#########################################################################
#
#  Lab 6-Image Processing
#  Main task is to pick and place all the blocks to your destination.
#  Camera is used to detect blocks. With image processing methods, 
#  robot is able to know the position, shape and orientation of the blocks.
#  Since block's position is based on image coordinate, what we should do
#  is to transform it into robot coordinate.
# 
#  In this lab, you only need to complete the image processing part.
#
#########################################################################

import cv2
import numpy as np
import math

class ImageProcess():
    def __init__(self):

        self.contours_elip = []
        self.contours_rect = []

    def template_process(self, img_path):

        # read image from img_path
        img_temp = cv2.imread(img_path)

        if img_temp is None:
            print('Template Image is None, Please check image path!')
            return
        
        img_copy = img_temp.copy()

        # Bilateral Filter smooths images while preserving edges,
        # by means of a nonlinear combination of nearby image values.
        # cv2.bilateralFilter(src, d, sigmaColor, sigmaSpace)
        # d - Diameter of each pixel neighborhood that is used during filtering.
        img_blur = cv2.bilateralFilter(img_copy, 19, 130, 30)

        # cv2.medianBlur() is used to reduce noise in the image
        img_blur = cv2.medianBlur(img_blur, 9)

        # cv2.cvtColor() is used to convert the image to grayscale
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

        # cv2.Sobel() is used to find the gradient of the image
        x_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 1, 0)
        y_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 0, 1)
        # cv2.Canny() is used to find the edges of the image
        img_canny = cv2.Canny(x_grid, y_grid, 30, 220)

        # split image into two parts, rectangle and ellipse
        weight, height = img_canny.shape
        img_rect = img_canny[:, :weight]
        img_elip = img_canny[:, weight:]

        # cv2.findContours to find contour
        # variable contours_1 stores all contours, each of which is composed of a series of pixel point 
        # For example: len(contours) contours[0] 
        # rectangle
        self.contours_rect, hierarchy = cv2.findContours(img_rect, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # elipse
        self.contours_elip, hierarchy = cv2.findContours(img_elip, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Test code
        cv2.imshow('canny image', img_canny)
        cv2.waitKey()
        cv2.destroyAllWindows()
        # print('contours_rect: ', self.contours_rect)
        # print('length of rectangle: ', len(self.contours_rect))
        # print('example of rectangle: ', self.contours_rect[0])
        # print('contours_elip: ', self.contours_elip)
        # print('length of ellipse: ', len(self.contours_elip))
        # print('example of ellipse: ', self.contours_elip[0])

    def image_process(self, img_path):

        # read image from img_path
        img_src = cv2.imread(img_path)

        if img_src is None:
            print('Source Image is None, Please check image path!')
            return
        
        img_copy = img_src.copy()

        ############## Your Code Start Here ##############
        # TODO: image process including filtering, edge detection, contour detection and so on.
        
        # Important: check contours and make sure the contour is available.
        # For one block, there will be 4 contours, 2 for rectangle or ellipse outside and 2 for arrow inside.
        # For one rectangle edge, there will be 2 contours in image resolution.
        # Tips: use cv2.contourArea(contour) as threshold to filter out the contour.

        img_blur = cv2.bilateralFilter(img_copy, 20, 50, 80)

        # cv2.medianBlur() is used to reduce noise in the image
        img_blur = cv2.medianBlur(img_blur, 7)

        # cv2.cvtColor() is used to convert the image to grayscale
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

        # cv2.Sobel() is used to find the gradient of the image
        x_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 1, 0)
        y_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 0, 1)
        # cv2.Canny() is used to find the edges of the image
        img_canny = cv2.Canny(x_grid, y_grid, 70, 150) # change the size ???

        # cv2.findContours to find contour
        # variable contours_1 stores all contours, each of which is composed of a series of pixel point
        # For example: len(contours) contours[0]
        # rectangle
        contours, hierarchy = cv2.findContours(img_canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        del_list = []
        for i in range(len(contours)):
            print(cv2.contourArea(contours[i]))
            if(cv2.contourArea(contours[i]) <= 40):
                del_list.append(i)

        #for i in range(len(del_list)-1,-1,-1):

        # Convert contours tuple to a list
        contours = list(contours)

        # Iterate over del_list to delete contours
        for i in range(len(del_list)-1, -1, -1):
            del contours[del_list[i]]

        # Convert contours back to a tuple if needed
        contours = tuple(contours)
        #del contours[del_list[i]]

        print("===================\n")
        for contour in contours:
            print(cv2.contourArea(contour))

        ############### Your Code End Here ###############

        # length of contours equals to 4 times the number of blocks
        print("length of contours: ", len(contours))

        ############## Your Code Start Here ##############
        # TODO: compute the center of contour and the angle of arrow,
        # as well as match shapes of your block
        center_value = []
        shape = [] # 0 represents rectangle while 1 represents ellipse
        theta = []
        for i in range(len(contours) // 2):
            if i % 2 == 0:
                ############## Your Code Start Here ##############
                # TODO: compute the center of external contour (rectangle/ellipse) and match shapes of your block
                # Tips: ret = cv2.matchShapes(contour1, contour2, 1, 0.0) 

                # cv2.circle(draw_img, (int(center_x), int(center_y)), 7, [0,0,255], -1)

                N = cv2.moments(contours[i * 2])
                _center_x = int(N["m10"] / N["m00"])
                _center_y = int(N["m01"] / N["m00"])
                center_value.append([_center_x,_center_y])
                # draw a circle on the center point
                cv2.circle(img_copy, (int(_center_x), int(_center_y)), 7, [0,0,255], -1)

                #shape.append(cv2.matchShapes(self.contours_elip[0], contours[i * 2], 1, 0.0))
                if(cv2.matchShapes(self.contours_elip[0], contours[2*i], 1, 0.0)<=cv2.matchShapes(self.contours_rect[0], contours[2*i], 1, 0.0)):
                    shape.append(1)
                else:
                    shape.append(0)
                pass

                ############### Your Code End Here ###############

            else:
                # compute the center of internal contour (arrow) and compute the angle of arrow
                N = cv2.moments(contours[i * 2])
                _center_x = int(N["m10"] / N["m00"])
                _center_y = int(N["m01"] / N["m00"])
                # draw a circle on the center point
                cv2.circle(img_copy, (int(_center_x), int(_center_y)), 7, [0,255,0], -1)

                ############## Your Code Start Here ##############
                # TODO: compute the angle of arrow
                # Tips: compute the distance between center point of external contour and every point of internal contour,
                # find the furthest point, then you can compute the angle.

                # cv2.line(draw_img, (_center_x, _center_y), (x, y), (128, 0, 0), 2)
                L = 0
                x = 0
                y = 0


                def distance(p1, p2):
                    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

                for point in contours[i*2]:
                    p = point[0]
                    temp = distance(p,center_value[i//2])
                    if (temp > L):
                        x = p[0]
                        y = p[1]
                        L = temp
                    #pass
                cv2.line(img_copy, (_center_x, _center_y), (x, y), (128, 0, 0), 2)

                angle = np.arctan2((y-_center_y)*1.0, (x-_center_x) * 1.0)
                theta.append(angle)

                ############### Your Code End Here ###############

        cv2.imshow('image with center and orientation', img_copy)
        cv2.waitKey()
        cv2.destroyAllWindows()

        return center_value, shape, theta

def coordinate_transform(center_cam, center_robot, center_snap):

    ############## Your Code Start Here ##############
    # TODO: transform camera coordinate to robot coordinate
    # a simple linear transformation is implemented refer to manual
    # Tip: center_value: [[Object1],[Object2],...]

    center_values = []

    x_ratio = (center_robot[1][0]-center_robot[0][0])/(float(center_cam[1][1])/1000-float(center_cam[0][1])/1000)
    y_ratio = (center_robot[1][1]-center_robot[0][1])/(float(center_cam[1][0])/1000-float(center_cam[0][0])/1000)

    for center in center_snap:
        x = center_robot[0][0] + x_ratio*(float(center[1])/1000-float(center_cam[0][1])/1000)
        y = center_robot[0][1] + y_ratio*(float(center[0])/1000-float(center_cam[0][0])/1000)
        center_values.append((x,y))

    return center_values
    # pass

    ############### Your Code End Here ###############

def lab_imgproc(center_robot1, center_robot2):

    # init image path
    # template image for template matching of rectangle and ellipse
    path_template = '/home/rvc/catkin_3220110826/src/lab7/lab7pkg_pick_place/img/template.jpg' 
    # two calibration images to calculate the image coordinate
    # with corresponding image coordinate and robot coordinate, 
    # a linear transformation between image coordinate and robot coordinate can be computed
    path_img_cali1 = '/home/rvc/catkin_3220110826/src/lab7/lab7pkg_pick_place/img/cali1.png'
    path_img_cali2 = '/home/rvc/catkin_3220110826/src/lab7/lab7pkg_pick_place/img/cali2.png'
    # snapshot image saved by your camera
    path_img_snap = '/home/rvc/catkin_3220110826/src/lab7/lab7pkg_pick_place/img/imgnew.png'

    # init ImageProcess class
    img_process = ImageProcess()

    # template process to get the contour of rectangle and ellipse
    img_process.template_process(path_template)

    # image process for calibration images to get the position of blocks
    center_cali1, shape_cali1, theta_cali1 = img_process.image_process(path_img_cali1)
    center_cali2, shape_cali2, theta_cali2 = img_process.image_process(path_img_cali2)

    # image process to get the shape, position and orientation of blocks
    center_snap, shape_snap, theta_snap = img_process.image_process(path_img_snap)

    # compute the coordinate of your blocks in robot coordinate system
    center_cam = []
    center_cam.append(center_cali1[0])
    center_cam.append(center_cali2[0])

    center_robot = []
    center_robot.append(center_robot1)
    center_robot.append(center_robot2)

    center_snap_robot = coordinate_transform(center_cam, center_robot, center_snap)

    return center_snap_robot, shape_snap, theta_snap
