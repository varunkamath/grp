#!/usr/bin/env python3

# Segment, recognize, and count fingers from a video feed

# imports
import cv2
import imutils
import numpy as np
import rospy
from sklearn.metrics import pairwise

# globals
bg = None


# Finding average background
def run_avg(image, accumWeight):
    global bg
    # initialize the background
    if bg is None:
        bg = image.copy().astype("float")
        return

    # compute weighted average, accumulate it and update the background
    cv2.accumulateWeighted(image, bg, accumWeight)


# Segmenting the hand using Otsu's Binarization + Gaussian filtering
def segment(image):
    global bg
    diff = cv2.absdiff(bg.astype("uint8"), image)

    # getting foreground
    blur = cv2.GaussianBlur(diff, (5, 5), 0)
    thresholded = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    # finding contours of the hand (theoretically)
    cnts, _ = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # robust error handling
    if len(cnts) == 0:
        print("this shit broke")
        return
    else:
        # returning hand contour (maximum)
        segmented = max(cnts, key=cv2.contourArea)
        return (thresholded, segmented)


# Counting fingers
def count(thresholded, segmented):
    # convex hull definition
    chull = cv2.convexHull(segmented)

    # finding extrema
    extreme_top = tuple(chull[chull[:, :, 1].argmin()][0])
    extreme_bottom = tuple(chull[chull[:, :, 1].argmax()][0])
    extreme_left = tuple(chull[chull[:, :, 0].argmin()][0])
    extreme_right = tuple(chull[chull[:, :, 0].argmax()][0])

    # finding center of area of the hand
    cX = int((extreme_left[0] + extreme_right[0]) / 2)
    cY = int((extreme_top[1] + extreme_bottom[1]) / 2)

    # finding max distance between hand COA and a fingertip
    distance = pairwise.euclidean_distances([(cX, cY)], Y=[extreme_left, extreme_right, extreme_top, extreme_bottom])[0]
    maximum_distance = distance[distance.argmax()]

    # defining circle (70% max distance)
    radius = int(0.7 * maximum_distance)
    circumference = (2 * np.pi * radius)

    # cropping to region of interest
    circular_roi = np.zeros(thresholded.shape[:2], dtype="uint8")

    # draw ROI
    cv2.circle(circular_roi, (cX, cY), radius, 255, 1)

    # bitwise 'and' to get segmented circle
    circular_roi = cv2.bitwise_and(thresholded, thresholded, mask=circular_roi)

    # finding contours of segmented circle
    (cnts, _) = cv2.findContours(circular_roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # loop through the contours found
    count = 0

    for c in cnts:
        # bounding box
        (x, y, w, h) = cv2.boundingRect(c)

        # increment the count of fingers only if -
        # 1. The contour region is not the wrist (bottom area)
        # 2. The number of points along the contour does not exceed
        #     25% of the circumference of the circular ROI
        if ((cY + (cY * 0.25)) > (y + h)) and ((circumference * 0.25) > c.shape[0]):
            count += 1

    print(count)
    return count, cY


if __name__ == "__main__":
    # initialize accumulated weights
    accumWeight = 0.5
    accumWeight2 = 0.5

    # getting video feed
    camera = cv2.VideoCapture(0)

    # defining handboxes
    top, right, bottom, left = 10, 350, 225, 590

    top2, right2, bottom2, left2 = 10, 60, 225, 300

    # calibration variables
    num_frames = 0
    calibrated = False
    calibrated2 = False

    # steering tolerance
    steer_tol = 15

    # action loop
    while (True):
        # get current frame
        (grabbed, frame) = camera.read()

        # resize
        frame = imutils.resize(frame, width=700)

        # flip frame
        frame = cv2.flip(frame, 1)

        # clone frame
        clone = frame.copy()
        clone2 = frame.copy()

        # hstack of feeds (testing)
        both = np.hstack((clone, clone2))

        # get height and width of the frame
        (height, width) = frame.shape[:2]

        # get regions of interest
        roi = frame[top:bottom, right:left]
        roi2 = frame[top2:bottom2, right2:left2]

        # convert to grayscale and blur
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        gray2 = cv2.cvtColor(roi2, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.GaussianBlur(gray2, (7, 7), 0)

        # to get the background, keep looking till a threshold is reached
        # so that our weighted average model gets calibrated
        if num_frames < 30:
            run_avg(gray, accumWeight)
            run_avg(gray2, accumWeight2)
            if num_frames == 1:
                print("[STATUS] please wait! calibrating...")
            elif num_frames == 29:
                print("[STATUS] calibration successfull...")
        else:
            # segment hand regions
            hand = segment(gray)
            hand2 = segment(gray2)

            # check whether regions segmented successfully
            if hand is not None and hand2 is not None:
                (thresholded, segmented) = hand
                (thresholded2, segmented2) = hand2

                # draw regions + frames
                cv2.drawContours(clone, [segmented + (right, top)], -1, (0, 0, 255))
                cv2.drawContours(clone, [segmented2 + (right2, top2)], -1, (0, 0, 255))

                # count fingers
                (fingers, area_y) = count(thresholded, segmented)
                (fingers2, area_y2) = count(thresholded2, segmented2)

                steer_num = area_y - area_y2
                steer = 'straight'

                if steer_num < -steer_tol:
                    steer = 'left'
                elif steer_num > steer_tol:
                    steer = 'right'

                cv2.putText(clone, str(fingers), (70, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(clone, str(fingers2), (360, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(clone, 'turning ' + str(steer), (15, 375), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # show thresholded images
                cv2.imshow("Right hand", thresholded)
                cv2.imshow("Left hand", thresholded2)

        # draw hands
        cv2.rectangle(clone, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.rectangle(clone, (left2, top2), (right2, bottom2), (0, 255, 0), 2)

        # increment the number of frames
        num_frames += 1

        # display frame with both hands
        cv2.imshow("Video Feed", clone)
        # cv2.imshow("Other hand dude", clone2)

        # watch for quit signal
        keypress = cv2.waitKey(1) & 0xFF

        if keypress == ord("q"):
            break

# free memory
camera.release()
cv2.destroyAllWindows()
