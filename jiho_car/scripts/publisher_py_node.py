#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

import cv2
import numpy as np
from csi_camera import CSI_Camera

import math

show_fps = False

# sonic_distance_1 = 0
# sonic_distance_2 = 0
# sonic_distance_3 = 0
# sonic_distance_4 = 0
# sonic_distance_5 = 0

# def cb_sonic_1(data):
#     sonic_distance_1 = data.data
#     print("sonic_1",sonic_distance_1)
# def cb_sonic_2(data):
#     sonic_distance_2 = data.data
#     #print(sonic_distance_1)
# def cb_sonic_3(data):
#     sonic_distance_3 = data.data
#     #print(sonic_distance_1)
# def cb_sonic_4(data):
#     sonic_distance_4 = data.data
#     #print(sonic_distance_1)
# def cb_sonic_5(data):
#     sonic_distance_5 = data.data
#     #print(sonic_distance_1)

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

def LaneDetection():
    pub = rospy.Publisher('/servo_jiho', Int16, queue_size=10)
    vel_pub = rospy.Publisher('/dc_jiho', Int16, queue_size=10)
    rospy.init_node('jiho_publisher_node', anonymous=True)
    # rospy.Subscriber("sonic_1", Float32, cb_sonic_1)
    # rospy.Subscriber("sonic_2", Float32, cb_sonic_2)
    # rospy.Subscriber("sonic_3", Float32, cb_sonic_3)
    # rospy.Subscriber("sonic_4", Float32, cb_sonic_4)
    # rospy.Subscriber("sonic_5", Float32, cb_sonic_5)
    rate = rospy.Rate(10) # 10hz
    
    print("jiwoo is beautiful.")
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
#        left_camera.start_counting_fps()
        while cv2.getWindowProperty("JiHo-Car Lane detection", 0) >= 0 :
            img=read_camera(left_camera,False)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            blur_gray = cv2.bilateralFilter(gray, 9, 75, 75)
            low_threshold = 60
            high_threshold = 150
            edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
            mask = np.zeros_like(edges)
            if len(edges.shape) > 2:
                channel_count = edges.shape[2]
                ignore_mask_color = (255,) * channel_count
            else:
                ignore_mask_color = 255
            vertices = np.array([[(0,360),(60,180),(580,180),(640,360)]], dtype=np.int32)
            cv2.fillPoly(mask, vertices, ignore_mask_color)
            masked_image = cv2.bitwise_and(edges, mask)

            #rho = 2
            #theta = np.pi/180
            #threshold = 90
            #min_line_len = 120
            #max_line_gap = 150

            rho = 1
            theta = np.pi/180
            threshold = 80
            min_line_len = 30
            max_line_gap = 30

            lines = cv2.HoughLinesP(masked_image, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            line_img = np.zeros((masked_image.shape[0], masked_image.shape[1], 3), dtype=np.uint8)

            position_left = []
            position_right = []

            if lines is None:
                print("lane is not detected!")
            else:
                jiwoo = 0
                # gradient_left = []
                # gradient_right = []
                for line in lines:
                    print(jiwoo, line)
                    for x1, y1, x2, y2 in line:
                        #cv2.line(line_img, (x1, y1), (x2, y2), [0,0,255], 5)
                        gradient = round((y2-y1)/float(x2-x1),4)
                        if gradient <= 0:
                            # gradient_left.append(gradient)
                            position_left.append((x1,y1,x2,y2))
                        else:
                            # gradient_right.append(gradient)
                            position_right.append((x1,y1,x2,y2))

                        # if x1 < 360 and y1 > 270:
                        #     gradient_left.append(round((y2-y1)/float(x2-x1),4))
                        # elif x2 > 360 and y2 > 270:
                        #     gradient_right.append(round((y2-y1)/float(x2-x1),4))
                    jiwoo += 1
            
            cv2.line(line_img, (310, 360), (310, 0), [0,255,0], 3)
            print("position_l", position_left)
            print("position_r", position_right)

            position_left_x1_avg = 0
            position_left_y1_avg = 0
            position_left_x2_avg = 0
            position_left_y2_avg = 0

            line_blind = 0
            
            for line in position_left:
                position_left_x1_avg += line[0] 
                position_left_y1_avg += line[1] 
                position_left_x2_avg += line[2] 
                position_left_y2_avg += line[3] 
            if len(position_left) != 0:
                position_left_x1_avg = position_left_x1_avg/len(position_left)
                position_left_y1_avg = position_left_y1_avg/len(position_left)
                position_left_x2_avg = position_left_x2_avg/len(position_left)
                position_left_y2_avg = position_left_y2_avg/len(position_left)
            else:
                position_left_x1_avg = 0
                position_left_y1_avg = 0
                position_left_x2_avg = 0
                position_left_y2_avg = 0
                line_blind = 1

            position_right_x1_avg = 0
            position_right_y1_avg = 0
            position_right_x2_avg = 0
            position_right_y2_avg = 0
            
            for line in position_right:
                position_right_x1_avg += line[0] 
                position_right_y1_avg += line[1] 
                position_right_x2_avg += line[2] 
                position_right_y2_avg += line[3] 
            if len(position_right) != 0:
                position_right_x1_avg = position_right_x1_avg/len(position_right)
                position_right_y1_avg = position_right_y1_avg/len(position_right)
                position_right_x2_avg = position_right_x2_avg/len(position_right)
                position_right_y2_avg = position_right_y2_avg/len(position_right)
            else:
                position_right_x1_avg = 0
                position_right_y1_avg = 0
                position_right_x2_avg = 0
                position_right_y2_avg = 0
                line_blind = 2

            print(position_left_x1_avg, position_left_y1_avg, position_left_x2_avg, position_left_y2_avg)
            print(position_right_x1_avg, position_right_y1_avg, position_right_x2_avg, position_right_y2_avg)

            cv2.line(line_img, (position_left_x1_avg, position_left_y1_avg), (position_left_x2_avg, position_left_y2_avg), [255,0,255], 7)
            cv2.line(line_img, (position_right_x1_avg, position_right_y1_avg), (position_right_x2_avg, position_right_y2_avg), [255,0,255], 7)

            if position_left_x2_avg-position_left_x1_avg != 0:
                gradient_left_avg = (position_left_y2_avg-position_left_y1_avg)/float(position_left_x2_avg-position_left_x1_avg)
            else:
                gradient_left_avg = 0
            if position_right_x2_avg-position_right_x1_avg != 0:
                gradient_right_avg = (position_right_y2_avg-position_right_y1_avg)/float(position_right_x2_avg-position_right_x1_avg)
            else:
                gradient_right_avg = 0
            
            vanish_y_offset = -65
            if line_blind == 0:
                if gradient_left_avg-gradient_right_avg != 0:
                    vanish_x = (gradient_left_avg*position_left_x1_avg-gradient_right_avg*position_right_x1_avg-position_left_y1_avg+position_right_y1_avg)/(gradient_left_avg-gradient_right_avg)
                else:
                    vanish_x = 310
            elif line_blind == 1:
                if gradient_right_avg != 0:
                    vanish_x = position_right_x1_avg + (vanish_y_offset-position_right_y1_avg)/gradient_right_avg
                else:
                    vanish_x = 310
            elif line_blind == 2:
                if gradient_left_avg != 0:
                    vanish_x = position_left_x1_avg + (vanish_y_offset-position_left_y1_avg)/gradient_left_avg
                else:
                    vanish_x = 310

            print(gradient_left_avg, gradient_right_avg)

            angle_add = 0
            add_gain = 10
            if gradient_left_avg == 0:
                angle_right = int(math.atan(gradient_right_avg)*180/math.pi)
                print("angle_right", angle_right)
                if angle_right > 50:
                    angle_add = -1*(angle_right - 50)*add_gain
            if gradient_right_avg == 0:
                angle_left = -1*int(math.atan(gradient_left_avg)*180/math.pi)
                print("angle_left", angle_left)
                if angle_left > 50:
                    angle_add = (angle_left - 50)*add_gain
            print("angle_Add", angle_add)
            vanish_x += angle_add
                    
            # blind_angle = 0
            # if gradient_left_avg == 0:    
            #     left_angle = blind_angle
            # else:
            #     left_angle = math.atan(left_avg)
            #     left_angle = -1*int(left_angle * 180 / math.pi)

            # if right_avg == 0:
            #     right_angle = -1*blind_angle
            # else:  
            #     right_angle = math.atan(right_avg)
            #     right_angle = -1*int(right_angle * 180 / math.pi) -1

            cv2.line(line_img, (310,360), (int(vanish_x), 180), [255,255,0], 3)
            cv2.circle(line_img, (int(vanish_x), 180), 5, [255,0,0], -1)
            print("vanish_x", vanish_x)

            angle_gain = 1/float(20)
            servo_angle = int((vanish_x - 310) * angle_gain)
            if servo_angle > 70:
                servo_angle = 70
            elif servo_angle < -70:
                servo_angle = -70
            print(servo_angle)
            pub.publish(servo_angle)

            mask_2 = np.zeros_like(edges)
            vertices_2 = np.array([[(0,135),(0,45),(640,45),(640,135)]], dtype=np.int32)
            cv2.fillPoly(mask_2, vertices_2, ignore_mask_color)
            masked_image_2 = cv2.bitwise_and(edges, mask_2)
            lines_2 = cv2.HoughLinesP(masked_image_2, rho, theta, 50, np.array([]), minLineLength=10, maxLineGap=30)
            line_img_2 = np.zeros((masked_image_2.shape[0], masked_image_2.shape[1], 3), dtype=np.uint8)
            gradient_left = []
            gradient_right = []
            if lines_2 is None:
                print("lane is not detected!")
            else:
                for line in lines_2:
                    for x1, y1, x2, y2 in line:
                        cv2.line(line_img_2, (x1, y1), (x2, y2), [0,0,255], 3)
                        gradient = round((y2-y1)/float(x2-x1),4)
                        if gradient <= 0:
                            gradient_left.append(gradient)
                        else:
                            gradient_right.append(gradient)
        
            #print("left", gradient_left)
            #print("right", gradient_right)
            
            if len(gradient_left) != 0:
                left_avg = sum(gradient_left)/len(gradient_left)
            else:
                left_avg = 0
            print("left_avg", left_avg)
            if len(gradient_right) != 0:
                right_avg = sum(gradient_right)/len(gradient_right)
            else:
                right_avg = 0
            print("right_Avg", right_avg)
            curve_angle = (right_avg-left_avg)/2
            print("curve_angle", curve_angle)

            if curve_angle > 0.8:
                velocity = 350
            elif curve_angle > 0.4:
                velocity = 300
            elif curve_angle > 0.2:
                velocity = 200
            else:
                velocity = 100
            
            vel_pub.publish(velocity)
            print("velocity", velocity)

            #print("sonic", sonic_distance_1, sonic_distance_2, sonic_distance_3, sonic_distance_4, sonic_distance_5)
            
            # gradient_avg = round(left_avg + right_avg, 4)/2
            # print("avg", gradient_avg)

            # servo_gain = 10*4*-1
            # pub.publish(int(servo_gain*gradient_avg))
            
            # blind_angle = 30
            # if left_avg == 0:    
            #     left_angle = blind_angle
            # else:
            #     left_angle = math.atan(left_avg)
            #     left_angle = -1*int(left_angle * 180 / math.pi)

            # if right_avg == 0:
            #     right_angle = -1*blind_angle
            # else:  
            #     right_angle = math.atan(right_avg)
            #     right_angle = -1*int(right_angle * 180 / math.pi) -1

            # angle_avg = (left_angle+right_angle)/2
            # pub.publish(angle_avg)
            
            # print("left", left_angle, "right", right_angle, "avg", angle_avg)

            line_asm = cv2.addWeighted(line_img, 1, line_img_2, 1, 0)
            lines_edgs = cv2.addWeighted(img, 0.8, line_asm, 1, 0)

            if show_fps:
                draw_label(img, "Frames Displayed (PS): "+str(left_camera.last_frames_displayed),(10,20))
                draw_label(img, "Frames Read (PS): "+str(left_camera.last_frames_read),(10,40))
            
            #cv2.imshow("edge",edges)
            #cv2.imshow("blur", blur_gray)

            #cv2.imshow("line_img", line_img)
            #cv2.imshow("line_img_2", line_img_2)
            
            cv2.imshow("JiHo-Car Lane detection", lines_edgs)
            
            left_camera.frames_displayed += 1
            keyCode = cv2.waitKey(5) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
    finally:
        left_camera.stop()
#        left_camera.release()
        cv2.destroyAllWindows()

def talker():
    pub = rospy.Publisher('/servo_jiho', Int16, queue_size=10)
    vel_pub = rospy.Publisher('/dc_jiho', Int16, queue_size=10)
    rospy.init_node('jiho_publisher_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #i = 1
    while not rospy.is_shutdown(): 
        rate.sleep()
        #vel_pub.publish(300)
        pub.publish(-7)
    # for i in range(20):
    #         msg = -i
    #         pub.publish(msg)
    #         rate.sleep()
    
    # while not rospy.is_shutdown(): 
    #     for i in range(30):
    #         msg = i
    #         pub.publish(msg)
    #         rate.sleep()
        # for i in range(60):
        #     msg = 30-i
        #     pub.publish(msg)
        #     rate.sleep()
        # for i in range(30):
        #     msg = -30+i
        #     pub.publish(msg)
        #     rate.sleep()
        
        #pub.publish(i)
        #i = i + 1
        #rate.sleep()

if __name__ == '__main__':
    try:
        LaneDetection()
        #talker()
    except rospy.ROSInterruptException:
        pass
