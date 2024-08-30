# MIT License
# Copyright (c) 2019 JetsonHacks
# See LICENSE for OpenCV license and additional information

# On the Jetson Nano, OpenCV comes preinstalled
# Data files are in /usr/sharc/OpenCV

import cv2
import numpy as np
from csi_camera import CSI_Camera

show_fps = True

# Simple draw label on an image; in our case, the video frame
def draw_label(cv_image, label_text, label_position):
    font_face = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    color = (255,255,255)
    # You can get the size of the string with cv2.getTextSize here
    cv2.putText(cv_image, label_text, label_position, font_face, scale, color, 1, cv2.LINE_AA)

# Read a frame from the camera, and draw the FPS on the image if desired
# Return an image
def read_camera(csi_camera,display_fps):
    _ , camera_image=csi_camera.read()
    if display_fps:
        draw_label(camera_image, "Frames Displayed (PS): "+str(csi_camera.last_frames_displayed),(10,20))
        draw_label(camera_image, "Frames Read (PS): "+str(csi_camera.last_frames_read),(10,40))
    return camera_image

# Good for 1280x720
DISPLAY_WIDTH=640
DISPLAY_HEIGHT=360
# For 1920x1080
# DISPLAY_WIDTH=960
# DISPLAY_HEIGHT=540

# 1920x1080, 30 fps
SENSOR_MODE_1080=2
# 1280x720, 60 fps
SENSOR_MODE_720=3

def face_detect():
    face_cascade = cv2.CascadeClassifier(
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
    )
    eye_cascade = cv2.CascadeClassifier(
        "/usr/share/opencv4/haarcascades/haarcascade_eye.xml"
    )
    left_camera = CSI_Camera()
    left_camera.create_gstreamer_pipeline(
            sensor_id=0,
            sensor_mode=SENSOR_MODE_720,
            framerate=30,
            flip_method=0,
            display_height=DISPLAY_HEIGHT,
            display_width=DISPLAY_WIDTH,
    )
    left_camera.open(left_camera.gstreamer_pipeline)
    left_camera.start()
    cv2.namedWindow("Face Detect", cv2.WINDOW_AUTOSIZE)

    if (
        not left_camera.video_capture.isOpened()
     ):
        # Cameras did not open, or no camera attached

        print("Unable to open any cameras")
        # TODO: Proper Cleanup
        SystemExit(0)
    try:
        # Start counting the number of frames read and displayed
        left_camera.start_counting_fps()
        while cv2.getWindowProperty("Face Detect", 0) >= 0 :
            img=read_camera(left_camera,False)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                roi_gray = gray[y : y + h, x : x + w]
                roi_color = img[y : y + h, x : x + w]
                eyes = eye_cascade.detectMultiScale(roi_gray)
                for (ex, ey, ew, eh) in eyes:
                    cv2.rectangle(
                        roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2
                    )
            if show_fps:
                draw_label(img, "Frames Displayed (PS): "+str(left_camera.last_frames_displayed),(10,20))
                draw_label(img, "Frames Read (PS): "+str(left_camera.last_frames_read),(10,40))
            cv2.imshow("Face Detect", img)
            left_camera.frames_displayed += 1
            keyCode = cv2.waitKey(5) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
    finally:
        left_camera.stop()
        left_camera.release()
        cv2.destroyAllWindows()

def LaneDetection():
    print("jiwoo is jingjing.")
    left_camera = CSI_Camera()
    left_camera.create_gstreamer_pipeline(
            sensor_id=0,
            sensor_mode=SENSOR_MODE_720,
            framerate=30,
            flip_method=0,
            display_height=DISPLAY_HEIGHT,
            display_width=DISPLAY_WIDTH,
    )
    left_camera.open(left_camera.gstreamer_pipeline)
    left_camera.start()
    cv2.namedWindow("JiHo-Car Lane detection", cv2.WINDOW_AUTOSIZE)
    
    if (
        not left_camera.video_capture.isOpened()
     ):
        # Cameras did not open, or no camera attached

        print("Unable to open any cameras")
        # TODO: Proper Cleanup
        SystemExit(0)
    try:
        # Start counting the number of frames read and displayed
        #left_camera.start_counting_fps()
        while cv2.getWindowProperty("JiHo-Car Lane detection", 0) >= 0 :
            img=read_camera(left_camera,False)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
            #sobel_x = cv2.convertScaleAbs(sobel_x)
            #sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1)
            #sobel_y = cv2.convertScaleAbs(sobel_y)
            #sobel_add = cv2.addWeighted(sobel_x, 1, sobel_y, 1, 0)
            #blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
            blur_gray = cv2.bilateralFilter(gray, 9, 75, 75)
            #sobel_x = cv2.Sobel(blur_gray, cv2.CV_64F, 1, 0)
            #sobel_x = cv2.convertScaleAbs(sobel_x)
            low_threshold = 60
            high_threshold = 150
            edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
            #print(img.shape[0])

            # mask = np.zeros_like(edges)
            # if len(edges.shape) > 2:
            #     channel_count = edges.shape[2]
            #     ignore_mask_color = (255,) * channel_count
            # else:
            #     ignore_mask_color = 255
            # vertices = np.array([[(0,edges.shape[0]),(0,180),(edges.shape[1],180),(edges.shape[1],edges.shape[0])]], dtype=np.int32)
            # cv2.fillPoly(mask, vertices, ignore_mask_color)
            # masked_image = cv2.bitwise_and(edges, mask)

            mask = np.zeros_like(edges)
            if len(edges.shape) > 2:
                channel_count = edges.shape[2]
                ignore_mask_color = (255,) * channel_count
            else:
                ignore_mask_color = 255
            vertices = np.array([[(0,180),(0,240),(edges.shape[1],180),(edges.shape[1],240)]], dtype=np.int32)
            cv2.fillPoly(mask, vertices, ignore_mask_color)
            masked_image = cv2.bitwise_and(edges, mask)

            #rho = 2
            #theta = np.pi/180
            #threshold = 90
            #min_line_len = 120
            #max_line_gap = 150

            rho = 1
            theta = np.pi/180
            threshold = 40
            min_line_len = 30
            max_line_gap = 10

            lines = cv2.HoughLinesP(masked_image, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            line_img = np.zeros((masked_image.shape[0], masked_image.shape[1], 3), dtype=np.uint8)
            if lines is None:
                print("lane is not detected!")
            else:
                jiwoo = 1
                print(len(lines))
                for line in lines:
                    print(jiwoo, line)
                    jiwoo += 1
                    for x1, y1, x2, y2 in line:
                        #print((y2-y1)/float((x2-x1)))
                        cv2.line(line_img, (x1, y1), (x2, y2), [0,0,255], 5)
            
            lines_edgs = cv2.addWeighted(img, 0.8, line_img, 1, 0)

            if show_fps:
                draw_label(img, "Frames Displayed (PS): "+str(left_camera.last_frames_displayed),(10,20))
                draw_label(img, "Frames Read (PS): "+str(left_camera.last_frames_read),(10,40))
            
            cv2.imshow("edge",edges)
            #cv2.imshow("masked", masked_image)
            #cv2.imshow("blur", blur_gray)
            #cv2.imshow("sobel", sobel_x)
            
            cv2.imshow("JiHo-Car Lane detection", lines_edgs)
            
            left_camera.frames_displayed += 1
            keyCode = cv2.waitKey(5) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
    finally:
        left_camera.stop()
        #left_camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    LaneDetection()
