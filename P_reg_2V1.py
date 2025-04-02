from __future__ import print_function
import cv2
import numpy as np
import time
import threading
from robolab_turtlebot import Turtlebot, Rate, get_time
from functions import depth_of_pixel, find_soccer_net_array, find_ball, find_ball_center
from math import sqrt, pi
import sys

turtle = Turtlebot(rgb=True, depth=True, pc=True)
picture_middle = 343
speed = 0.2
global close_flag
close_flag = False

def find_ball_xcenter(hsv_img):
    ball = find_ball(hsv_img)
    
    if (ball is not None):
        print("Length of ball", len(ball))
        b_coords = find_ball_center(ball)
        bx = b_coords[0]
        by = b_coords[1]
    else: 
        print("bx = None")
        bx = None
    return bx,ball


def finding_leftmost_point(cnt):
    x_coords = [point[0][0] for point in cnt]  # Extrahujeme X souřadnice
    min_x = min(x_coords)  # Najdeme nejmenší X souřadnici
    return min_x

def finding_righttmost_point(cnt):
    x_coords = [point[0][0] for point in cnt]  # Extrahujeme X souřadnice
    max_x = max(x_coords)  # Najdeme nejmenší X souřadnici
    return max_x

def find_width_of_ball(cnt):
    x_coords = [point[0][0] for point in cnt]  # Extrahujeme X souřadnice
    min_x = min(x_coords)  # Najdeme nejmenší X souřadnici
    max_x = max(x_coords)  # Najdeme nejmenší X souřadnici
    width = max_x-min_x
    return width

def goalnet_center_by_one_pillar(p1x,bx,cnt):
    if(p1x>bx):
        min_x = finding_leftmost_point(cnt)
        half_of_pillar = p1x - min_x
        middle_goalnet = p1x-11*half_of_pillar#11 in case, that pillar width is 7cm and goalnet width is 70 cm
        return middle_goalnet
    else:
        max_x = finding_righttmost_point(cnt)
        half_of_pillar = max_x-p1x
        middle_goalnet = p1x+11*half_of_pillar
        return middle_goalnet


def find_goalnet_center(hsv_img,bx):
    soccer_net_array, x_pillar, y_pillar = find_soccer_net_array(hsv_img)
    if(x_pillar[0] != None and x_pillar[1] != None):
        xp1 = x_pillar[0]
        xp2 = x_pillar[1]
        mpx = (xp1+xp2)/2
    elif (x_pillar[0] != None and x_pillar[1] == None):
        mpx = goalnet_center_by_one_pillar(x_pillar[0],bx,soccer_net_array[0])
    else:
        mpx = None
    return(mpx)


def take_hsv_picture():
    while True:
        turtle.wait_for_rgb_image()
        bgr_image = turtle.get_rgb_image()
        #print(bgr_image.shape)
        break
    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV) #prevod do HSV

def go_straight(meters):
    print("GO_straight")
    t = get_time()
    final_spped = 2
    time = abs(meters/final_spped)
    rate = Rate(100)
    
    while get_time() - t < time:
        print("provadim")
        turtle.cmd_velocity(linear=final_spped)
        rate.sleep()
    
    #sys.exit()

def go_straight_odom(wanted_dist):
    turtle.reset_odometry() 
    turtle.wait_for_odometry()
    rate = Rate(100)  # 100 Hz smyčka řízení

    while True:
        #Iteration value
        print("GO_STRAIGHT_ODOM")

        x, y, _angle = turtle.get_odometry()
        travelled_dist = sqrt(x**2 + y**2)

        if abs(wanted_dist - travelled_dist) < 0.01:
            turtle.cmd_velocity(linear=0, angular=0)
            break

        turtle.cmd_velocity(linear=2, angular=0)

        rate.sleep()

    #sys.exit()

def rotate_to_center_ball(): 
    
    rate = Rate(100)
    turtle.reset_odometry()
    turtle.wait_for_odometry()

    while True:
        hsv_img = take_hsv_picture()
        ball = find_ball(hsv_img)

        if ball is None:
            turtle.cmd_velocity(angular=0.6)
            rate.sleep()
        else:
            b_coords = find_ball_center(ball)
            print(b_coords[0]-320)
            if abs(320 - b_coords[0]) < 5:
                turtle.cmd_velocity(angular=0)
                break
            else:
                speed = 0.3
                if abs(b_coords[0] - 320) < 40:
                    speed = 0.15

                elif abs(b_coords[0] - 320) < 20:
                    speed = 0.05

                if b_coords[0] > 320:
                    speed = -speed

                turtle.cmd_velocity(angular=speed)

    return turtle.get_odometry()



def ball_far_far_away(bx,hsv_img):#main thing is to get ball center to the center of goalnet
    
    rad_per_pixel = 1.63*10**(-3)#pi/3/640
    max_ang_speed = 2

    if(bx!= None):
        mpx = find_goalnet_center(hsv_img,bx)


    if(mpx != None):
        delta_x = (mpx - bx)
        ampl_factor_ang =  rad_per_pixel * delta_x
        ampl_factor_ang = max(-max_ang_speed, min(ampl_factor_ang, max_ang_speed))
        ampl_factor_lin = max(0.05, min(1.0, picture_middle / (picture_middle + abs(delta_x))))

        turtle.cmd_velocity(linear = ampl_factor_lin*speed, angular=3*ampl_factor_ang)
    else:
        print("bx and mpx are None")


def ball_close(bx):#main thing is to hit ball straight
    print("BALL_CLOSE STARTED")
    global close_flag
    close_flag = True
    rad_per_pixel = 1.63*10**(-3)#pi/3/640
    max_ang_speed = 2

    
    delta_x = (picture_middle - bx)
    ampl_factor_ang =  rad_per_pixel * delta_x
    ampl_factor_ang = max(-max_ang_speed, min(ampl_factor_ang, max_ang_speed))
    #ampl_factor_lin = picture_middle/(picture_middle+abs(delta_x))
    ampl_factor_lin = max(0.05, min(1.0, picture_middle / (picture_middle + abs(delta_x))))

    turtle.cmd_velocity(linear = ampl_factor_lin*speed, angular=2*ampl_factor_ang)

def rotate_to_find_ball():
    ball_not_in_middle = True

    while(ball_not_in_middle):
        hsv_img = take_hsv_picture()
        bx,ball = find_ball_xcenter(hsv_img)
        if(bx != None):
            delta_x = (picture_middle - bx)
            if(delta_x < 4):
                ball_not_in_middle = False
        
        turtle.cmd_velocity(linear = 0, angular=0.3)




def hit_ball():#+angular is left
    go_now = True
    #close_flag = False
    rate = Rate(100)
    #rotate_to_find_ball()
    rotate_to_center_ball()
    while(go_now == True):
        hsv_img = take_hsv_picture()
        bx,ball = find_ball_xcenter(hsv_img)
        if(bx== None and close_flag == True):
            go_straight(0.1)
            #go_straight_odom(0.1)
            break
        elif(bx == None):
            #rotate_to_find_ball()
            rotate_to_center_ball()

        #print(ball)
        #if( ball != None):
        else:
            b_width = find_width_of_ball(ball)
            if(b_width < (640/6)):
                ball_far_far_away(bx,hsv_img)
            else:
                ball_close(bx)
        """         if(len(ball) > 0):
            b_width = find_width_of_ball(ball)
            if(b_width < (640/5)):
                ball_far_far_away(bx,hsv_img)
            else:
                ball_close(bx)
        """


        rate.sleep()

    














def main():
    hit_ball()



if __name__ == "__main__":
    main()
