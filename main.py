from __future__ import print_function
import cv2 as cv
import numpy as np
from math import atan2, cos, sin, sqrt, pi

#TH for aligning image
MAX_FEATURES = 5000
GOOD_MATCH_PERCENT = 0.05#0.15

imFilename = "QRTest.jpg"

def FindAng(ImgInput):
    cv.imwrite("PicForAng.jpg", ImgInput)

    # Load the image
    img = cv.imread("PicForAng.jpg")

    # Was the image there?
    if img is None:
        print("Error: File not found")
        exit(0)

    #cv.imshow('Input Image', img)

    # Convert image to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Convert image to binary
    _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

    # Find all the contours in the thresholded image
    contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

    listang = []

    for i, c in enumerate(contours):

        # Calculate the area of each contour
        area = cv.contourArea(c)

        # Ignore contours that are too small or too large
        if area < 2000 or 1000000 < area:
            continue

        # cv.minAreaRect returns:
        #(center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(c)
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)

        # Retrieve the key parameters of the rotated bounding box
        centerx = int(rect[0][0])
        centery = int(rect[0][1])
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])

        while(angle < 0):
            angle = angle + 90

        while(angle > 90):
            angle = angle - 90

        listang.append(angle)

    return listang

def alignImages(im1, im2):
    # Convert images to grayscale
    im1Gray = cv.cvtColor(im1, cv.COLOR_BGR2GRAY)
    im2Gray = cv.cvtColor(im2, cv.COLOR_BGR2GRAY)

    # Detect ORB features and compute descriptors.
    orb = cv.ORB_create(MAX_FEATURES)
    keypoints1, descriptors1 = orb.detectAndCompute(im1Gray, None)
    keypoints2, descriptors2 = orb.detectAndCompute(im2Gray, None)

    # Match features.
    matcher = cv.DescriptorMatcher_create(cv.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
    matches = matcher.match(descriptors1, descriptors2, None)

    # Sort matches by score
    matches.sort(key=lambda x: x.distance, reverse=False)

    # Remove not so good matches
    numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]

    # Draw top matches
    imMatches = cv.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
    cv.imwrite("matches.jpg", imMatches)

    # Extract location of good matches
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)

    for i, match in enumerate(matches):
        points1[i, :] = keypoints1[match.queryIdx].pt
        points2[i, :] = keypoints2[match.trainIdx].pt

    # Find homography
    h, mask = cv.findHomography(points1, points2, cv.RANSAC)

    # Use homography
    height, width, channels = im2.shape
    im1Reg = cv.warpPerspective(im1, h, (width, height))

    return im1Reg, h

#------------------------------------------------------------------------------------------------------------Aligning and crop image------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    # Read reference image
    refFilename = "Reff25.png"
    print("Reading reference image : ", refFilename)
    imReference = cv.imread(refFilename, cv.IMREAD_COLOR)

    # Read image to be aligned
    print("Reading image to align : ", imFilename);
    im = cv.imread(imFilename, cv.IMREAD_COLOR)

    print("Aligning images ...")
    # Registered image will be resotred in imReg.
    # The estimated homography will be stored in h.
    imReg, h = alignImages(im, imReference)

    # Write aligned image to disk.
    outFilename = "aligned.jpg"
    print("Saving aligned image : ", outFilename);
    cv.imwrite(outFilename, imReg)

#------------------------------------------------------------------------------------------------------------Blobs------------------------------------------------------------------------------------------------------------

ColorNames = ["Orange","Green","Blue","White","Yellow","LBlue"]

#image blur
image=cv.imread(outFilename)
image_blur = cv.blur(image,(15,15))

#BGR in to HSV
hsv = cv.cvtColor(image_blur, cv.COLOR_BGR2HSV)

#HSV TRESHOLD (ISOLATING ORANGE)
# lower boundary ORANGE color range values; Hue (0 - 10)
lowerORANGE = np.array([3,170,165],np.uint8)
upperORANGE = np.array([10,255,240],np.uint8)
full_maskORANGE = cv.inRange(hsv, lowerORANGE, upperORANGE)

#HSV TRESHOLD (ISOLATING LIGHT BLUE)
# lower boundary LIGHT BLUE color range values; Hue (0 - 10)
lowerLBLUE = np.array([80, 119, 109],np.uint8)
upperLBLUE = np.array([95, 255, 161],np.uint8)
full_maskLBLUE = cv.inRange(hsv, lowerLBLUE, upperLBLUE)

#HSV TRESHOLD (ISOLATING BLUE)
# lower boundary BLUE color range values; Hue (0 - 10)
lowerBLUE = np.array([81,90,52],np.uint8)
upperBLUE = np.array([98,215,157],np.uint8)
full_maskBLUE = cv.inRange(hsv, lowerBLUE, upperBLUE)

#HSV TRESHOLD (ISOLATING GREEN)
# lower boundary GREEN color range values; Hue (0 - 10)
lowerGREEN = np.array([30,149,62],np.uint8)
upperGREEN = np.array([41,255,186],np.uint8)
full_maskGREEN = cv.inRange(hsv, lowerGREEN, upperGREEN)

#HSV TRESHOLD (ISOLATING YELLOW)
# boundary YELLOW color range values; Hue (0 - 10)
lowerYELLOW = np.array([17,239,71])
upperYELLOW = np.array([29, 255, 255])
full_maskYELLOW = cv.inRange(hsv,lowerYELLOW,upperYELLOW)

#HSV TRESHOLD (ISOLATING WHITE)
# boundary WHITE color range values; Hue (0 - 10)
lowerWHITE = np.array([0,200,48], dtype=np.uint8)
upperWHITE = np.array([4,255,190], dtype=np.uint8)
full_maskWHITE = cv.inRange(hsv,lowerWHITE,upperWHITE)

#FIXING COLOURING
maskLIST=[full_maskORANGE,full_maskGREEN,full_maskBLUE,full_maskWHITE,full_maskYELLOW,full_maskLBLUE]
mask = []
for i in maskLIST:
    mask.append(cv.dilate(i, None, iterations=2))

#REVERSE MASK
revmask = []
for i in range(len(mask)):
    revmask.append(255-mask[i])
    #cv.imshow("Mask",revmask[i])
    #cv.waitKey(0)
cv.imwrite("MaskOriTest.jpg", revmask[1])

#DETECTOR
params = cv.SimpleBlobDetector_Params()

params.minThreshold = 10
params.maxThreshold = 255

params.filterByArea = True
params.minArea = 2000
params.maxArea = 1000000

params.filterByCircularity = True
params.minCircularity = 0.500

params.filterByConvexity = False
params.minConvexity = 0.87

params.filterByInertia = False
params.minInertiaRatio = 0.01

detector = cv.SimpleBlobDetector_create(params)
keypoints = []
for i in range(len(revmask)):
    keypoints.append(detector.detect(revmask[i]))

#DRAWING KEYPOINTS
im_w_keypoints = []
for i in range(len(keypoints)):
    im_w_keypoints.append(cv.drawKeypoints(revmask[i], keypoints[i], np.array([]), (255, 255, 0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS))
    cv.imshow(ColorNames[i], im_w_keypoints[i])
    cv.waitKey(0)
#------------------------------------------------------------------------------------------------------------Add orientation------------------------------------------------------------------------------------------------------------

AngList = []
for i in revmask:
    AngList.append(FindAng(i))
#------------------------------------------------------------------------------------------------------------Make brick list------------------------------------------------------------------------------------------------------------
Brick = []

#Contert px to mm

for x in range(len(keypoints)):
    list = []
    for i in range(len(keypoints[x])):
        #print(keypoints[x][i].pt[0],"-", keypoints[x][i].pt[1])
        cord = [int(keypoints[x][i].pt[0]), int(keypoints[x][i].pt[1])]
        cord.append(99)
        cord.append(AngList[x][i])
        list.append(cord)
    Brick.append(list)


print(Brick)
names = ['brick1.csv','brick2.csv','brick3.csv','brick4.csv','brick5.csv','brick6.csv']

import pandas as pd
import os
for i in range(len(Brick)):
    os.getcwd()
    df = pd.DataFrame(Brick[i])
    df.to_csv(names[i])

import csv
newList = []
for name in names:
    with open(name) as csvfile:
        reader = csv.reader(csvfile)
        col1 = []

        for row in reader:
            col1.append(row)

        col1.pop(0)
        for list in col1:
            list.pop(0)
        for i in col1:
            for x in range(len(i)):
                i[x] = int(i[x])
        newList.append(col1)
print(newList)