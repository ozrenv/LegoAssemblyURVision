import cv2 as cv
import numpy as np

  image    = cv2.blur("LEGOtest.jpg", (blur, blur))
        #- Show result
        if imshow:
            cv2.imshow("Blur", mask)
            cv2.imshow("Blur", image)
            cv2.waitKey(0)

    #- Search window
    if search_window is None: search_window = [0.0, 0.0, 1.0, 1.0]
@@ -51,17 +52,24 @@ def blob_detect(image,                  #-- The frame (cv standard)

    #- dilate makes the in range areas larger
    mask = cv2.dilate(mask, None, iterations=2)
    #- Show HSV Mask
    if imshow:
        cv2.imshow("Dilate Mask", mask)
        cv2.waitKey(0)

    mask = cv2.erode(mask, None, iterations=2)

    #- Show dilate/erode mask
    if imshow:
        cv2.imshow("Dilate/Erode Mask", mask)
        cv2.imshow("Erode Mask", mask)
        cv2.waitKey(0)

    #- Cut the image using the search mask
    mask = cut_image(mask, search_window)
    mask = apply_search_window(mask, search_window)

    if imshow:
        cv2.imshow("Cropping Mask", mask)
        cv2.imshow("Searching Mask", mask)
        cv2.waitKey(0)

    #- build default blob detection parameters, if none have been provided
    if blob_params is None:
@@ -100,6 +108,7 @@ def blob_detect(image,                  #-- The frame (cv standard)

    if imshow:
        cv2.imshow("Reverse Mask", reversemask)
        cv2.waitKey(0)

    keypoints = detector.detect(reversemask)

@@ -173,7 +182,7 @@ def draw_frame(image,

#---------- Apply search window: returns the image
#-- return(image)
def cut_image(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px    = int(cols*window_adim[0])
@@ -237,8 +246,8 @@ def get_blob_relative_position(image, keyPoint):
    window = [0.25, 0.25, 0.65, 0.75]

    #-- IMAGE_SOURCE: either 'camera' or 'imagelist'
    SOURCE = 'video'
    #SOURCE = 'camera'
    #SOURCE = 'video'
    SOURCE = 'camera'

    if SOURCE == 'video':
        cap = cv2.VideoCapture(0)
@@ -263,23 +272,29 @@ def get_blob_relative_position(image, keyPoint):
        #-- Read image list from file:
        image_list = []
        image_list.append(cv2.imread("blob.jpg"))
        image_list.append(cv2.imread("blob2.jpg"))
        image_list.append(cv2.imread("blob3.jpg"))
        #image_list.append(cv2.imread("blob2.jpg"))
        #image_list.append(cv2.imread("blob3.jpg"))

        for image in image_list:
            #-- Detect keypoints
            keypoints, _ = blob_detect(image, blue_min, blue_max, blur=5,
                                        blob_params=None, search_window=window, imshow=False)
            #-- Draw search window
            image     = draw_window(image, window, imshow=True)

            #-- click ENTER on the image window to proceed
            draw_keypoints(image, keypoints, imshow=True)
                                        blob_params=None, search_window=window, imshow=True)

            #-- enter to proceed, q to quit
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
            image    = blur_outside(image, blur=15, window_adim=window)
            cv2.imshow("Outside Blur", image)
            cv2.waitKey(0)

            image     = draw_window(image, window, imshow=True)
            #-- enter to proceed
            cv2.waitKey(0)

            #-- click ENTER on the image window to proceed
            image     = draw_keypoints(image, keypoints, imshow=True)
            cv2.waitKey(0)
            #-- Draw search window


            image    = draw_frame(image)
            cv2.imshow("Frame", image)
            cv2.waitKey(0)
