#!/usr/bin/env python

'''

lab7pkg_pick_place/lab7_img.py

@brief: image processing for lab6, including template matching, image filtering, edge detection, contour detection, etc.
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

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
        # cv2.imshow('canny image', img_canny)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
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

        img_blur = cv2.bilateralFilter(img_copy, 19, 130, 30)
        img_blur = cv2.medianBlur(img_blur, 9)
        img_gray = cv2.cvtColor(img_blur, cv2.COLOR_BGR2GRAY)

        x_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 1, 0)
        y_grid = cv2.Sobel(img_gray, cv2.CV_16SC1, 0, 1)
        img_canny = cv2.Canny(x_grid, y_grid, 30, 220)

        raw_contours, hierarchy = cv2.findContours(img_canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if hierarchy is None:
            contours = []
        else:
            hierarchy = hierarchy[0]  # (N,4): [next, prev, child, parent]

            # area filter (tune if needed)
            min_area = 200.0
            max_area = float(img_src.shape[0] * img_src.shape[1]) * 0.5

            ext = []
            inn = []
            for idx, c in enumerate(raw_contours):
                a = cv2.contourArea(c)
                if a < min_area or a > max_area:
                    continue
                parent = hierarchy[idx][3]
                if parent == -1:
                    ext.append(c)
                else:
                    inn.append(c)

            # pair each external contour with one internal contour inside it (arrow)
            contours = []
            for ce in sorted(ext, key=cv2.contourArea, reverse=True):
                xe, ye, we, he = cv2.boundingRect(ce)

                best_ci = None
                best_score = None
                for ci in inn:
                    xi, yi, wi, hi = cv2.boundingRect(ci)
                    if (xi >= xe and yi >= ye and (xi + wi) <= (xe + we) and (yi + hi) <= (ye + he)):
                        score = cv2.contourArea(ci)
                        if best_score is None or score > best_score:
                            best_score = score
                            best_ci = ci

                if best_ci is None:
                    continue

                # order must support later indexing: contours[i*2] and each duplicated
                contours.extend([ce, ce, best_ci, best_ci])

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

                N = cv2.moments(contours[i * 2])
                center_x = int(N["m10"] / N["m00"])
                center_y = int(N["m01"] / N["m00"])
                center_value.append([center_x, center_y])

                temp_rect = max(self.contours_rect, key=cv2.contourArea) if len(self.contours_rect) > 0 else None
                temp_elip = max(self.contours_elip, key=cv2.contourArea) if len(self.contours_elip) > 0 else None

                if temp_rect is None and temp_elip is None:
                    shape.append(0)
                elif temp_elip is None:
                    shape.append(0)
                elif temp_rect is None:
                    shape.append(1)
                else:
                    r_rect = cv2.matchShapes(contours[i * 2], temp_rect, 1, 0.0)
                    r_elip = cv2.matchShapes(contours[i * 2], temp_elip, 1, 0.0)
                    shape.append(0 if r_rect < r_elip else 1)

                cv2.circle(img_copy, (int(center_x), int(center_y)), 7, [0,0,255], -1)

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

                ext_cx, ext_cy = center_value[-1][0], center_value[-1][1]

                pts = contours[i * 2].reshape(-1, 2)
                dx = pts[:, 0] - ext_cx
                dy = pts[:, 1] - ext_cy
                dist2 = dx * dx + dy * dy
                k = int(np.argmax(dist2))
                x, y = int(pts[k, 0]), int(pts[k, 1])

                ang = float(np.arctan2(y - _center_y, x - _center_x))  # radians
                theta.append(ang)

                cv2.line(img_copy, (_center_x, _center_y), (x, y), (128, 0, 0), 2)

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

    x1, y1 = center_cam[0][0], center_cam[0][1]
    x2, y2 = center_cam[1][0], center_cam[1][1]

    x1p, y1p = center_robot[0][0], center_robot[0][1]
    x2p, y2p = center_robot[1][0], center_robot[1][1]

    xratio = (x2p - x1p) / (y2 - y1)
    yratio = (y2p - y1p) / (x2 - x1)

    center_snap_robot = []
    for c in center_snap:
        x, y = c[0], c[1]
        xp = x1p + xratio * (y - y1)
        yp = y1p + yratio * (x - x1)
        center_snap_robot.append([xp, yp])

    return center_snap_robot

    ############### Your Code End Here ###############

def lab_imgproc(center_robot1, center_robot2):

    # init image path
    # template image for template matching of rectangle and ellipse
    path_template = "../img/template.jpg"
    # two calibration images to calculate the image coordinate
    # with corresponding image coordinate and robot coordinate,
    # a linear transformation between image coordinate and robot coordinate can be computed
    path_img_cali1 = '../img/img_cali_1.bmp'
    path_img_cali2 = '../img/img_cali_2.bmp'
    # snapshot image saved by your camera
    path_img_snap = '../img/img_snap.bmp'

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
