from __future__ import print_function
import cv2
import numpy as np
import threading
from robolab_turtlebot import Turtlebot, Rate, get_time
from math import pi, sqrt
from time import sleep

from calc import *
from movements import *
from functions import *

# Initialize the Turtlebot instance
turtle = Turtlebot(rgb=True, depth=True, pc=True)
picture_middle = 343
speed = 0.2
close_flag = False

def button_event(msg):
    global event
    event.set()

def take_hsv_picture():
    while True:
        turtle.wait_for_rgb_image()
        bgr_image = turtle.get_rgb_image()
        break
    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

#alternative
def second_task():

    event.wait()  # wait for button push
    
    success = False

    while not success:

        if rotate_to_center_net() is None:
            if rotate_to_center_ball(precision=50) is None:
                print("ERROR: Unable to find ball nor the pillars. Exiting.")
                exit(1)

            ball_dst = measure_ball_dst()

            if ball_dst > 0.5:
                incremental_go_straight(ball_dst - 0.5, 1, turtle)
            else:
                rotate(pi/2, turtle)
                rotate(pi/2, turtle)
                incremental_go_straight(1, 1, turtle)

            if rotate_to_center_net() is None:
                print("ERROR: Unable to find the net. Exiting.")
                exit(1)
        
        pos = get_robot_pos()

        go_to_pos(pos, [0, 2.5, 0], turtle)
        go_behind_the_ball()
    
        if hit_ball():
            success = True

def go_behind_the_ball():
    if rotate_to_center_net() is None:
        print("ERROR: Unable to find the net. Exiting.")
        exit(1)

    pos = get_robot_pos()
    rot = rotate_to_center_ball(precision=30)
    if rot is None:
        print("ERROR: Robot can't see the ball. Exiting.")
        exit(1)

    pos[2] += rot[2] #we have to update rotation
    rel_ball_pos = scan_ball_pos(pos, turtle) #this may fail
    ball_pos = [pos[0] + rel_ball_pos[0], pos[1] - rel_ball_pos[1], 0]

    pos_behind = pos_behind_the_ball(ball_pos, 1.5)
    go_to_pos(pos, pos_behind, turtle)

 
def hit_ball():#+angular is left
    print("----------------------------------Hit_ball")
    go_now = True
    rate = Rate(100)
    delta_x = None
    rotate_to_center_ball()
    
    while(go_now == True):
        hsv_img = take_hsv_picture()
        bx, ball = find_ball_xcenter(hsv_img)
        if bx== None and close_flag == True :
            go_straight(0.3, turtle)
            break
        elif(bx == None):
            rotate_to_center_ball()

        else:
            b_width = find_width_of_ball(ball)
            if(b_width < (640/6)):
                delta_x = ball_far_far_away(bx,hsv_img)
            elif(delta_x != None and delta_x < 35): #35 pixels difference is not good enough for our goal
                ball_close(bx)
            else:
                print("robot couldnt hanlde the regulation so it shoud start from beginning")
                turtle.play_sound(6)
                return False
        rate.sleep()

    return True
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


#def second_task():
#    event.wait()
#    pos = get_robot_pos()
#
#    #pos = count_robot_pos()
#
#    go_to_pos(pos, [0, 2, 0], turtle)
#    go_to_pos([0,2,0], [0, 0, 0], turtle)


def rotate_to_center_ball(precision = 10): 
    
    rate = Rate(100)
    turtle.reset_odometry()
    turtle.wait_for_odometry()
    ball_seen = False
    t = get_time()

    while True:
        hsv_img = take_hsv_picture()
        ball = find_ball(hsv_img)

        #we haven't spotted the ball in 20 sec; end
        if not ball_seen and get_time() - t > 20:
            return None

        if ball is None:
            turtle.cmd_velocity(angular=0.6)
            rate.sleep()

        else:
            ball_seen = True
            b_coords = find_ball_center(ball)
            print(b_coords[0]-320)
            if abs(320 - b_coords[0]) < precision:
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

#we assume that the rotation is known and noted in curr_pos
def scan_ball_pos(curr_pos, turtle): 
    hsv_img = take_hsv_picture()
    ball = find_ball(hsv_img)

    if ball is None:
        return None
    
    #we can compute the position of the ball
    b_coords = find_ball_center(ball)
    print("souřadnice míče v obrázku", *b_coords)

    return count_relative_ball_pos(b_coords, curr_pos, turtle)

def measure_ball_dst():
    hsv_img = take_hsv_picture()
    ball = find_ball(hsv_img)
    if ball is None:
        return None
    b_coords = find_ball_center(ball)
    point_cloud = turtle.get_point_cloud()
    dst = hood_depth(point_cloud, *b_coords)
    return dst

def rotate_to_center_net():

    for _ in range(18):
        hsv_image = take_hsv_picture()
        net, x_c, y_c = find_soccer_net_array(hsv_image)

        if net[0] is not None and net[1] is not None:
            return True

        simple_rotate(pi/8, turtle)

    return None

#def count_robot_pos():
#
#    successful = 0
#    parts = 16
#    overall_pos = [0, 0, 0]
#
#    for i in range(parts):
#        rotate(2*pi/parts, turtle)
#
#        pos = get_robot_pos()
#
#        if pos is None:
#            continue
#
#        pos[2] -= i*2*pi/parts
#
#        successful += 1
#        overall_pos[0] += pos[0]
#        overall_pos[1] += pos[1]
#        overall_pos[2] += pos[2]
#
#    if successful < 1:
#        return None
#
#    return [overall_pos[0]/pictures_to_take,
#            overall_pos[1]/pictures_to_take,
#            overall_pos[2]/pictures_to_take]

def count_robot_pos():    
    if rotate_to_center_net() is None:
        return None

    successful = 0
    overall_pos = [0, 0, 0]
    segments = 9

    for i in range(segments):
        pos = get_robot_pos()
        if pos is None:
            break
        successful += 1
        pos[2] -= (i*pi)/(2*segments)
        print(pos[2])
        overall_pos[0] += pos[0]
        overall_pos[1] += pos[1]
        overall_pos[2] += pos[2]
        rotate(pi/(2*segments), turtle)

    return [overall_pos[0]/successful,
            overall_pos[1]/successful,
            overall_pos[2]/successful + successful*pi/(2*segments)]


#from one position, without moving in space
#def count_robot_pos():
#    successful = 0
#    pictures_to_take = 3 #take three pictures
#
#    if rotate_to_center_net() is None:
#        return None
#
#    overall_pos = [0, 0, 0]
#
#    for i in range(pictures_to_take):
#        pos = get_robot_pos()
#        if pos is not None:
#            overall_pos[0] += pos[0]
#            overall_pos[1] += pos[1]
#            overall_pos[2] += pos[2]
#            successful += 1
#    print("Úspěchů:", successful) 
#    if successful/pictures_to_take < 2/3:
#        print("Malý podíl úspěšných fotek.")
#        return None
#
#    return [overall_pos[0]/pictures_to_take,
#            overall_pos[1]/pictures_to_take,
#            overall_pos[2]/pictures_to_take]

def get_robot_pos(): #or None if it cannot see the pillars

    hsv_image = take_hsv_picture()
    turtle.wait_for_point_cloud()
    point_cloud = turtle.get_point_cloud()
    
    net, x_c, y_c = find_soccer_net_array(hsv_image)
    
    if net[0] is None or net[1] is None:
        return None
    
    dl = hood_depth(point_cloud, x_c[0], y_c[0])
    dr = hood_depth(point_cloud, x_c[1], y_c[1])

    #distance of pillars
    dp = two_pts_real_dst([x_c[0], y_c[0]], [x_c[1], y_c[1]], point_cloud)
    
    if dp is None:
        print("pillar dst not found")
    else:
        print('distance of pillars: ', dp)

    return count_pos(dl, dr, dp, x_c)


def main():
    global event
    event = threading.Event()
    turtle.register_bumper_event_cb(button_event)
    event.set()
    
    # Start the second task in a separate thread
    program_thread = threading.Thread(target=second_task)
    program_thread.start()
    
    rate = Rate(5)
    while not turtle.is_shutting_down():
        rate.sleep()
    
    # Ensure the thread stops when the program ends
    event.set()
    program_thread.join()

if __name__ == "__main__":
    main()

